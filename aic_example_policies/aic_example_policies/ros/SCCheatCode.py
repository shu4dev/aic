import math
import os

import numpy as np

from aic_example_policies.utils.wrench_smoother import (
    DEFAULT_MODE as WRENCH_DEFAULT_MODE,
    DEFAULT_WINDOW_S as WRENCH_DEFAULT_WINDOW_S,
    WrenchSmoother,
)
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_control_interfaces.msg import (
    JointMotionUpdate,
    TrajectoryGenerationMode,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import TransformException
from trajectory_msgs.msg import JointTrajectoryPoint
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

QuaternionTuple = tuple[float, float, float, float]

CONNECTOR_PARAMS = {
    "sfp": {
        # Lowered from -0.022 → -0.030 (8 mm deeper). The cheatcode commands
        # the plug to z_offset=max_depth at the bottom of descent; once the
        # plug physically bottoms out at the port's hard floor, additional
        # commanded depth becomes pure position error for the impedance
        # controller (Kp_z=90 N/m → ~0.72 N per 8 mm of error). Spot-check
        # of partial trials showed the cable stopping a few mm above the
        # port floor — the extra push force seats them. JAM detection at
        # F_limit=15 N still retracts safely if we overshoot a real bind.
        "max_depth": -0.030,
        "coarse_step": 0.0012,
        "fine_step": 0.0003,
        "fine_threshold": 0.01,
        "force_limit": 15.0,
        "force_ref": 4.0,
        # Bumped 5 → 8 N in Exp-27 smoke iteration (2026-05-07). 5N was
        # right when CHEATCODE_DESCENT_OFFSET_XY caused deliberate port-lip
        # binding, but with abs() force-trigger and bias-augmented cheatcode,
        # 5N also fires on normal port-rim rubbing during good insertion —
        # which then triggers retreat via directed correction. 8N keeps
        # backoff to genuine sustained card-press cases.
        "force_soft": 8.0,
        # SFP-specific behavior knobs (matches current V2 descent semantics).
        # SFP has a flat card-top to press against, so abs() force trigger
        # catches the case where the gripper body lands on the card and the
        # vector magnitude drops below baseline. Big 20 mm backoff lift gets
        # the cable cleanly off the port lip into open air for lateral
        # retarget. Approach handoff is trusted (no explicit align gate)
        # because SFP's ~2 mm cage tolerance is forgiving.
        "force_trigger_abs": True,
        "backoff_lift_m": 0.020,
        "align_after_approach": False,
        "align_max_iters": 0,
        "align_xy_threshold": 0.0015,
        "align_plug_vel_threshold": 0.0003,
        # Control stiffness/damping for align+settle+descend. None falls
        # through to set_pose_target's 90/50 default — the value SFP was
        # tuned around. Override per-trial with CHEATCODE_STIFFNESS_HIGH=1.
        "control_stiffness": None,
        "control_damping": None,
        "settle_iters": 20,
        # In-approach PI lock-in: when > 0, the last this-fraction of
        # _approach() iterations run with the PI integrator active and
        # high stiffness, so approach exits with the plug already
        # aligned. 0 keeps SFP's two-phase (cosine-only approach → no
        # alignment) — its wide cage tolerates an unconverged hand-off.
        "approach_converge_frac": 0.0,
    },
    "sc": {
        "max_depth": -0.035,
        "coarse_step": 0.0005,
        "fine_step": 0.0003,
        "fine_threshold": 0.008,
        "force_limit": 12.0,
        "force_ref": 3.5,
        # Restored 4.0 → 7.0 to match V1. The V2 drop to 4 N was tuned for
        # SFP card-press semantics; for SC there's no card-top, the spring-
        # loaded ferrule generates sustained seating force in the final
        # ~1 mm of legitimate insertion, and 4 N fires backoff partway
        # through a good insert. 7 N keeps backoff for genuine binding only.
        "force_soft": 7.0,
        # SC-specific behavior knobs (restores V1 semantics on the parts of
        # V2's descent logic that were tuned for SFP).
        #   force_trigger_abs=False: SC has no card-top to press against.
        #     Using |Δf| just causes false backoffs on transient magnitude
        #     dips. V1 used signed delta; restore that here.
        #   backoff_lift_m=0.005: SC seats through a tight alignment sleeve
        #     where every backoff destroys partial-engagement progress.
        #     V1's `-base_step * 0.5` was a tiny vertical retreat (~0.6 mm
        #     at coarse_step). 5 mm is a compromise — enough to clear lip
        #     contact, small enough not to lose alignment.
        #   align_after_approach=True: SC's tight tolerance means descent
        #     from a settle that hasn't fully converged is doomed. Use V1's
        #     stability gate (xy < 1.5 mm AND plug_vel < 0.3 mm/iter for 3
        #     consecutive iters) before allowing descent.
        "force_trigger_abs": False,
        "backoff_lift_m": 0.005,
        # align_after_approach=False here because alignment now happens
        # inside _approach via approach_converge_frac (below). The
        # separate align loop in _align_and_descend stays as fallback
        # for tasks that prefer two-phase, but is dead code for SC.
        "align_after_approach": False,
        "align_max_iters": 300,   # ~15 s cap at 20 Hz (unused for SC now)
        "align_xy_threshold": 0.0015,
        "align_plug_vel_threshold": 0.0003,
        # SC needs V1's 400/200 profile: the impedance controller at the
        # 90/50 default cannot generate enough lateral force for the PI
        # integrator wind-up to drag the cable through the alignment
        # sleeve — plug_vel stays at ~0.04 mm/iter and align never
        # converges, leaving the descent to bind on the port lip.
        "control_stiffness": [400.0, 400.0, 400.0, 150.0, 150.0, 150.0],
        "control_damping":  [200.0, 200.0, 200.0,  60.0,  60.0,  60.0],
        # V1 ran 40 iters (~1.6 s) of cable damping before descent; the
        # 20-iter ACT-chunk-window constraint only matters when
        # downstream-policy chunk semantics drive the schedule.
        "settle_iters": 40,
        # Last 40 % of _approach() iters run PI-active with the 400/200
        # stiffness above. That folds V1's separate alignment phase into
        # approach: by the time _approach() returns, the integrator has
        # wound up and dragged the cable to within align_xy_threshold of
        # the port, so settle/descent start aligned instead of 5–48 mm
        # off-center. Stage 1 (first 60 %) is unchanged — cosine blend
        # with integrator reset every iter — preserving the existing
        # low-jerk swing-minimized trajectory shape.
        "approach_converge_frac": 0.4,
        # Drop in wrench magnitude below baseline (N) that counts as
        # sustained lip-support contact. Signed-delta `force_trigger_abs=
        # False` clamps df_med to max(0, force-baseline), so when the
        # cable rests on the port lip (lip supports cable weight → wrench
        # drops ~12-14 N below baseline) backoff never fires and descent
        # walks z to max_depth with the plug frozen at zgap=13.47 mm.
        # 8 N is well above transient dip noise (~3-4 N) but below the
        # observed sustained lip-support drop, so normal seating doesn't
        # false-trigger.
        "force_drop": 8.0,
        # Tighten the off-center gate for SC. The default 2 mm is SFP-
        # tuned (wide cage); SC's alignment sleeve clearance is sub-mm,
        # so 1.5-2 mm cable_to_port still means "on the lip", not "in
        # the hole". Without this, the off-center gate blocks backoff
        # when bias happens to align closely (e.g., xy_to_port=1.79 mm).
        "backoff_distance_gate_m": 0.0008,
    },
}
DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]

# Cap collision-and-retry attempts; if exceeded, abort the trial. Without
# this, a worst-case bind could spin the descent loop forever. Bumped from
# 5 → 8 after observing ~55% of trials hit the cap before finding a working
# offset at sigma=1.5mm — the extra attempts convert most partials → full
# insertion at little marginal cost (each attempt adds ~1-2s).
MAX_DESCENT_BACKOFF_ATTEMPTS = 8


class SCCheatCode(Policy):
    def __init__(self, parent_node):
        self._task = None
        self._ix = 0.0
        self._iy = 0.0
        self._max_windup = 0.05
        self._i_gain = 0.15
        self._approach_only = os.environ.get("CHEATCODE_APPROACH_ONLY", "0") == "1"
        # Control-loop rate. Determines how often CheatCode emits
        # `_send_pose` commands during approach / align / settle / descent
        # / end-hold. All downstream pipeline rates (training-dataset fps,
        # RunACT ACT_CONTROL_HZ at eval) should match this so the
        # (state, action) cadence the model sees at training matches
        # what it will emit at eval. Default 20 sim Hz to match the AIC
        # sim camera's 20-sim-Hz unique-content rate (camera is the
        # bottleneck). Read once at __init__; loop sleeps via
        # self.sleep_for which uses the node's sim-time-aware clock.
        # Only control-tick sleeps use this; TF-retry waits and the
        # post-approach / JAM settling delays stay at their hardcoded
        # values intentionally.
        self._control_hz = float(os.environ.get("CHEATCODE_HZ", "20.0"))
        self._control_dt = 1.0 / self._control_hz
        # Optional uniform high-stiffness profile for ablation experiments:
        # when CHEATCODE_STIFFNESS_HIGH=1, _send_pose defaults stiffness to
        # 400/150 and damping to 200/60 (matches yujin's per-phase override).
        # Default off → falls through to set_pose_target's 90/50 baseline,
        # which matches RunACT's current eval-time stiffness.
        self._high_stiffness = os.environ.get("CHEATCODE_STIFFNESS_HIGH", "0") == "1"
        # DART-style noise injection: add Gaussian noise to commanded pose
        # before publishing. Causes the robot to drift off the ideal path,
        # which creates off-path (state, action) pairs in the training data
        # so the downstream policy sees recovery behavior. The PI integrator
        # naturally corrects on the next iteration.
        #   CHEATCODE_NOISE_POS — 1-sigma noise in each position axis (meters)
        #   CHEATCODE_NOISE_ROT — 1-sigma small-angle noise magnitude (radians)
        self._noise_pos = float(os.environ.get("CHEATCODE_NOISE_POS", "0.001") or 0.0)
        self._noise_rot = float(os.environ.get("CHEATCODE_NOISE_ROT", "0.01") or 0.0)
        # Each worker uses its own RNG so parallel collections don't correlate.
        self._rng = np.random.default_rng()
        # Descent target xy noise — when > 0, the descent target is biased by
        # a per-attempt Gaussian offset (sticky during one descent attempt).
        # Causes the plug to engage the port off-center, producing real
        # binding events that fire backoff and demonstrate collision-and-retry
        # behavior in the rollout. Resampled on each backoff event so the next
        # attempt explores a different position. Set to ~σ_aperture (port
        # half-width, e.g. 1.5 mm for SFP) for a useful mix of clean inserts
        # and binding events.
        self._descent_offset_sigma = float(
            os.environ.get("CHEATCODE_DESCENT_OFFSET_XY", "0.0015") or 0.0
        )
        # Approach-handoff biases — when sigma > 0, every episode samples a
        # sticky XY/rotation offset that shifts the approach + aligning +
        # settling target. The descending phase then re-aligns to the true
        # port frame before descending. Result: training data covers
        # "settle off-center, then find the port" — the OOD the hier_actact
        # specialist saw at eval handoff (approach lands at ~15-25mm XY
        # error, specialist trained on <6mm). Sticky per *trial*: actual
        # bias values are sampled at the start of each insert_cable() call.
        #   CHEATCODE_APPROACH_XY_BIAS  — 1-σ XY position bias (meters)
        #   CHEATCODE_APPROACH_ROT_BIAS — 1-σ rotation bias (radians)
        self._approach_xy_sigma = float(
            os.environ.get("CHEATCODE_APPROACH_XY_BIAS", "0.005") or 0.0
        )
        self._approach_rot_sigma = float(
            os.environ.get("CHEATCODE_APPROACH_ROT_BIAS", "0.02") or 0.0
        )
        # Per-trial bias values — set by _sample_approach_biases() at the
        # start of each insert_cable() call. Identity defaults so any code
        # path that runs before sampling still gets clean targets.
        self._approach_xy_bias = (0.0, 0.0)
        self._approach_rot_bias = (1.0, 0.0, 0.0, 0.0)  # identity (w,x,y,z)
        super().__init__(parent_node)
        # ReentrantCallbackGroup shared across all cheatcode subscriptions
        # (wrench, /scoring/tf, ...). Created here, BEFORE WrenchSmoother,
        # so we can pass it down. See longer comment near _scoring_tf_sub.
        from rclpy.callback_groups import ReentrantCallbackGroup
        self._cb_group = ReentrantCallbackGroup()
        # Wrench smoother — subscribes to /fts_broadcaster/wrench directly so
        # we can read freshest force samples without going through the
        # camera-rate-limited Observation. Window/mode default to the same
        # values mcap_to_lerobot uses when building the dataset, so the
        # contact dynamics CheatCode reacts to here are the same the model
        # sees during training.
        self._wrench_smoother = WrenchSmoother(parent_node, callback_group=self._cb_group)
        self._wrench_window_s = float(
            os.environ.get("WRENCH_SMOOTH_WINDOW_S", str(WRENCH_DEFAULT_WINDOW_S))
        )
        self._wrench_mode = os.environ.get("WRENCH_SMOOTH_MODE", WRENCH_DEFAULT_MODE)

        # Phase event publisher — emits std_msgs/Header at every phase
        # transition (approach, aligning, settling, descending, done). The
        # frame_id field carries the phase name; consumers (notably
        # scripts/phase_to_slice_csv.py) match against frame_id == "descending"
        # to mark the approach→insertion handoff used for insertion-specialist
        # training data slicing. The publisher lives on the parent_node (not
        # the lifecycle node) so it persists across deactivate/activate
        # cycles between trials — required for the engine's pre-trial
        # endpoint check on /aic_cheatcode/phase. Default QoS (reliable,
        # volatile, depth=10) — explicitly NOT TRANSIENT_LOCAL so a previous
        # trial's events don't replay into the next bag.
        self._phase_pub = parent_node.create_publisher(
            Header, "/aic_cheatcode/phase", 10
        )
        # Event publisher — emits structured backoff/jam events on a
        # SEPARATE topic so the per-phase stream stays clean for existing
        # consumers (phase_to_slice_csv.py matches phase names exactly).
        # Each event is a Header with stamp=sim time + frame_id encoding
        # the event details, e.g. "backoff:N=3:to_port=2.95mm:dx=-0.48,dy=-1.03mm".
        # Format is simple key=value pairs joined by ':' so analysis scripts
        # can split on ':' and parse without a custom msg type.
        self._event_pub = parent_node.create_publisher(
            Header, "/aic_cheatcode/events", 10
        )

        # Direct /scoring/tf subscription. Bypasses the
        # topic_tools-relay-then-TF-buffer pipeline, which empirically
        # delivers cable transforms 40+ mm stale (verified by comparing
        # /scoring/tf vs /tf in the same bag — sfp_tip_link lagged the
        # ground truth by tens of mm at end-of-trial). We feed each
        # transform directly into the existing TF buffer with our own
        # authority name; lookup_transform() then sees fresh data.
        # tf2_ros.Buffer is thread-safe.
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos_scoring = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        # Use the shared ReentrantCallbackGroup created earlier (alongside
        # WrenchSmoother). All cheatcode subscriptions share this group so
        # they get their own executor-thread pool, decoupled from the
        # default mutex group's queue (lifecycle/parameter/action callbacks).
        self._scoring_tf_sub = parent_node.create_subscription(
            TFMessage, "/scoring/tf", self._on_scoring_tf, qos_scoring,
            callback_group=self._cb_group,
        )
        self._scoring_tf_n = 0

        # Maintain a raw map of (parent, child) -> Transform from /scoring/tf
        # so we can compose the cable/port chains manually without going
        # through tf2_ros.Buffer.lookup_transform — which empirically uses
        # "latest common time" semantics that are bottlenecked by the
        # stalest leg in the chain (often the multi-second-stale port leg).
        # Bypassing the buffer for cable+port gives us truly-latest data.
        self._scoring_tfs = {}  # (parent, child) -> (stamp_ns, Transform)
        # Cache of base_link -> aic_world (static after sim start) so we
        # can compose aic_world-frame cable/port transforms back into
        # base_link without further buffer queries each call.
        self._cached_base_to_aic = None  # 4x4 matrix
        # NOTE: not subscribing to /tf_static directly — caused 15s
        # _wait_for_tf timeouts in earlier smoke (suspected QoS or buffer
        # interference). The TF buffer's internal /tf_static listener
        # already populates static frames; tf2_ros.Buffer.set_transform
        # for /scoring/tf data is the only injection we do here.

    def _on_scoring_tf(self, msg) -> None:
        """Receive transforms from /scoring/tf (and /tf_static).

        Two effects:
          1. Stash raw transforms in self._scoring_tfs for our own chain
             composition (bypasses tf2_ros.Buffer's "latest common time"
             chain semantics — see _plug_tf/_port_tf).
          2. Also feed the buffer with set_transform so other consumers
             still work; we just don't trust the buffer for cable+port.
        Runs on a reentrant callback group, parallel to the main loop.
        """
        try:
            for tf in msg.transforms:
                stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                            + tf.header.stamp.nanosec)
                self._scoring_tfs[(tf.header.frame_id, tf.child_frame_id)] = (
                    stamp_ns, tf.transform,
                )
                self._parent_node._tf_buffer.set_transform(
                    tf, "cheatcode_scoring_tf"
                )
            self._scoring_tf_n += 1
        except Exception as exc:  # noqa: BLE001
            # Defensive: don't kill the callback thread on any single bad msg.
            if self._scoring_tf_n % 1000 == 0:
                self.get_logger().warn(f"scoring_tf feed exception: {exc}")

    @staticmethod
    def _tf_to_matrix(t):
        """geometry_msgs/Transform -> 4x4 numpy matrix."""
        qw, qx, qy, qz = t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z
        n = qw*qw + qx*qx + qy*qy + qz*qz
        if n < 1e-12:
            R = np.eye(3)
        else:
            s = 2.0 / n
            xx, yy, zz = qx*qx*s, qy*qy*s, qz*qz*s
            xy, xz, yz = qx*qy*s, qx*qz*s, qy*qz*s
            wx, wy, wz = qw*qx*s, qw*qy*s, qw*qz*s
            R = np.array([
                [1-(yy+zz), xy-wz,    xz+wy],
                [xy+wz,    1-(xx+zz), yz-wx],
                [xz-wy,    yz+wx,     1-(xx+yy)],
            ])
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
        return M

    @staticmethod
    def _matrix_to_tf(M):
        """4x4 numpy -> geometry_msgs/Transform."""
        from geometry_msgs.msg import Transform, Vector3, Quaternion
        R = M[:3, :3]
        tx, ty, tz = float(M[0, 3]), float(M[1, 3]), float(M[2, 3])
        tr = R[0,0] + R[1,1] + R[2,2]
        if tr > 0:
            S = 2.0 * math.sqrt(tr + 1.0)
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
        return Transform(
            translation=Vector3(x=tx, y=ty, z=tz),
            rotation=Quaternion(w=float(qw), x=float(qx), y=float(qy), z=float(qz)),
        )

    def _compose_aic_world_to(self, target_frame):
        """Walk parent chain from target_frame back to aic_world using
        self._scoring_tfs, return 4x4 aic_world -> target_frame matrix.
        Returns (matrix, oldest_stamp_ns) or (None, 0) if chain incomplete.
        """
        # Build parent-of map from current dict snapshot
        parent_of = {child: parent for (parent, child) in self._scoring_tfs}
        chain_children = []
        f = target_frame
        # Walk back at most 20 hops (any cable/port chain is much shorter)
        for _ in range(20):
            if f == "aic_world":
                break
            if f not in parent_of:
                return None, 0
            chain_children.append(f)
            f = parent_of[f]
        else:
            return None, 0  # no aic_world ancestor within 20 hops
        if f != "aic_world":
            return None, 0
        # Compose aic_world -> target by multiplying parent-to-child legs
        # in order from aic_world outward
        chain_children.reverse()
        M = np.eye(4)
        oldest = 1 << 62
        for c in chain_children:
            p = parent_of[c]
            stamp_ns, tf = self._scoring_tfs[(p, c)]
            M = M @ self._tf_to_matrix(tf)
            oldest = min(oldest, stamp_ns)
        return M, oldest

    def _bypass_lookup(self, target_frame):
        """Like _lookup('base_link', target_frame) but composes from our
        /scoring/tf-fed dict for the aic_world->target leg, multiplied
        by a one-shot cached base_link->aic_world from the TF buffer.
        Returns Transform or None.
        """
        # Cache base_link -> aic_world once. It's static for a fixed-base
        # robot + static_transform_publisher world->aic_world.
        if self._cached_base_to_aic is None:
            try:
                tf_static = self._parent_node._tf_buffer.lookup_transform(
                    "base_link", "aic_world", Time())
                self._cached_base_to_aic = self._tf_to_matrix(tf_static.transform)
            except TransformException:
                return None  # buffer not ready yet
        M_aic_to_target, oldest = self._compose_aic_world_to(target_frame)
        if M_aic_to_target is None:
            return None
        # Stash freshness for events.
        try:
            now_ns = self._parent_node.get_clock().now().nanoseconds
            self._last_tf_age_s = max(0.0, (now_ns - oldest) / 1e9)
        except Exception:
            pass
        return self._matrix_to_tf(self._cached_base_to_aic @ M_aic_to_target)

    def _wait_for_tf(self, target, source, timeout_sec=15.0):
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(target, source, Time())
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(f"Waiting for TF '{source}' -> '{target}'...")
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(f"TF '{source}' not available after {timeout_sec}s")
        return False

    # Stale-TF guard. The TF buffer's `lookup_transform(..., Time())` happily
    # returns a 1-minute-old cached transform with no warning — silently
    # producing wrong cable→port readings if the bridge or sim has hiccuped.
    # We compare the returned stamp to wall time and warn on stale reads,
    # debounced to avoid flooding the log when staleness persists.
    _STALE_TF_THRESHOLD_S = 0.10   # 100 ms — warn if returned tf older than this
    _STALE_TF_LOG_INTERVAL_S = 5.0  # only re-log a stale source every N seconds

    def _lookup(self, target, source):
        try:
            tf = self._parent_node._tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        # Compute stamp age vs. node clock. Both use ROS time (sim time when
        # use_sim_time:=true, wall otherwise) so they're directly comparable.
        try:
            now = self._parent_node.get_clock().now()
            stamp = Time.from_msg(tf.header.stamp)
            age_s = (now - stamp).nanoseconds / 1e9
        except Exception:
            age_s = -1.0  # unknown — skip warning
        # Stash for events to embed (latest lookup wins).
        self._last_tf_age_s = age_s
        if age_s > self._STALE_TF_THRESHOLD_S:
            if not hasattr(self, '_stale_log_last'):
                self._stale_log_last = {}
            wall_now = self._now_s()
            last = self._stale_log_last.get(source, 0.0)
            if wall_now - last >= self._STALE_TF_LOG_INTERVAL_S:
                self._stale_log_last[source] = wall_now
                # Diagnose: which leg of the chain is bottlenecking the
                # latest_common_time? Print the latest-known stamp for each
                # candidate intermediate frame, so we can see which one
                # tf2 is using as the chain's "latest common time."
                detail = self._diagnose_chain(target, source)
                self.get_logger().warn(
                    f"STALE TF: {source} returned stamp age {age_s*1000:.0f}ms "
                    f"(threshold {self._STALE_TF_THRESHOLD_S*1000:.0f}ms){detail}"
                )
        return tf.transform

    def _diagnose_chain(self, target, source):
        """Inspect the TF chain and report each leg's age. Best-effort —
        returns empty string if introspection isn't supported."""
        try:
            buf = self._parent_node._tf_buffer
            # Common intermediate frames in the chain we care about
            chain_frames = [
                target,
                "world",
                "aic_world",
                "task_board",
                "cable_0",
                source,
            ]
            now = self._parent_node.get_clock().now()
            ages = []
            for f in chain_frames:
                # latest stamp for any transform whose CHILD is `f`
                try:
                    if buf.can_transform(target, f, Time()):
                        common = buf.get_latest_common_time(target, f)
                        age_ms = (now - common).nanoseconds / 1e6
                        ages.append(f"{f}={age_ms:.0f}ms")
                    else:
                        ages.append(f"{f}=N/A")
                except Exception:
                    ages.append(f"{f}=err")
            return f"  chain_ages={{{', '.join(ages)}}}"
        except Exception as e:
            return f"  diagnose_err={e}"

    def _port_tf(self):
        # Try the bypass path first (fresh /scoring/tf data composed via
        # cached static base_link->aic_world). Fall back to buffer if our
        # dict doesn't have the chain yet (e.g., before first /scoring/tf).
        bypass = self._bypass_lookup(self._port_frame)
        if bypass is not None:
            return bypass
        return self._lookup("base_link", self._port_frame)

    def _plug_tf(self):
        bypass = self._bypass_lookup(self._plug_frame)
        if bypass is not None:
            return bypass
        return self._lookup("base_link", self._plug_frame)

    def _grip_tf(self):
        # Gripper stays on the buffer — its chain is purely robot URDF
        # (no relayed external frames), and earlier diagnostics showed
        # only ~100ms staleness, well within tolerance.
        return self._lookup("base_link", "gripper/tcp")

    def _q(self, rot):
        return (rot.w, rot.x, rot.y, rot.z)

    def _get_force(self, get_obs):
        obs = get_obs()
        if obs is None or obs.wrist_wrench is None:
            return 0.0
        f = obs.wrist_wrench.wrench.force
        return math.sqrt(f.x**2 + f.y**2 + f.z**2)

    def _now_s(self) -> float:
        """Sim-time-aware wall clock as a float (seconds)."""
        return self.time_now().nanoseconds * 1e-9

    def _sample_approach_biases(self) -> None:
        """Draw new XY + rotation handoff biases for this trial.

        Called once at the start of insert_cable(). When sigmas are 0 these
        are identities and approach behaves as before. Stored on self so
        _approach + aligning + settling all see the same biased target,
        and the re-align block in descending sees identity again.
        """
        if self._approach_xy_sigma > 0:
            self._approach_xy_bias = (
                float(self._rng.normal(0, self._approach_xy_sigma)),
                float(self._rng.normal(0, self._approach_xy_sigma)),
            )
        else:
            self._approach_xy_bias = (0.0, 0.0)

        if self._approach_rot_sigma > 0:
            axis = self._rng.normal(size=3)
            n = float(np.linalg.norm(axis))
            if n > 1e-9:
                axis /= n
                angle = float(self._rng.normal(0, self._approach_rot_sigma))
                half = 0.5 * angle
                s = math.sin(half)
                self._approach_rot_bias = (
                    math.cos(half),
                    float(axis[0] * s),
                    float(axis[1] * s),
                    float(axis[2] * s),
                )
                rot_deg = abs(angle) * 180.0 / math.pi
            else:
                self._approach_rot_bias = (1.0, 0.0, 0.0, 0.0)
                rot_deg = 0.0
        else:
            self._approach_rot_bias = (1.0, 0.0, 0.0, 0.0)
            rot_deg = 0.0

        self.get_logger().info(
            f"[approach_bias] xy=({self._approach_xy_bias[0]*1000:+.1f},"
            f"{self._approach_xy_bias[1]*1000:+.1f})mm  rot={rot_deg:.2f}deg"
        )

    def _publish_phase(self, name: str) -> None:
        """Emit a phase event on /aic_cheatcode/phase with frame_id=name.

        Stamp uses sim time so phase boundaries align with everything else
        in the bag (camera frames, joint states, etc.) when post-processing.
        """
        msg = Header()
        msg.stamp = self.time_now().to_msg()
        msg.frame_id = name
        self._phase_pub.publish(msg)
        self.get_logger().info(f"[phase] {name}")

    def _publish_event(self, payload: str) -> None:
        """Emit a structured event on /aic_cheatcode/events.

        Payload is the full event string (e.g.
        "backoff:N=3:to_port=2.95mm:dx=-0.48,dy=-1.03mm"); it goes verbatim
        into Header.frame_id. Persists in the bag so analysis can reconstruct
        backoff sequences even after the cheatcode log is truncated.
        """
        msg = Header()
        msg.stamp = self.time_now().to_msg()
        msg.frame_id = payload
        self._event_pub.publish(msg)

    def _measure_baseline(self, get_obs, n=10):
        """Average force magnitude over a 200ms window before descent.

        Reads from the wrench smoother's ring buffer (fed by the direct
        /fts_broadcaster/wrench subscription, ~100Hz) rather than polling
        get_obs (which is camera-rate-limited to ~20Hz). Returns the mean of
        the magnitudes of all in-window samples — same channel as
        peak_magnitude/median_magnitude so threshold semantics line up.
        """
        # Wait long enough for the buffer to fill at least 100 ms of fresh
        # samples (ported from V5). At ~100 Hz wrench rate that's still ≥10
        # medians — plenty for robust statistics — and shaves 100 ms of
        # "stay still" labels off every trial.
        self.sleep_for(0.1)
        now = self._now_s()
        # Use a 200ms causal window — wider than the per-iter control window
        # (50ms) so we average across a couple of camera cycles.
        peak = self._wrench_smoother.peak_magnitude(now, window_s=0.2)
        median = self._wrench_smoother.median_magnitude(now, window_s=0.2)
        # Baseline uses median (robust). Peak printed for diagnostics.
        if median == 0.0:
            # Fallback — smoother had nothing yet. One get_obs call.
            return self._get_force(get_obs)
        self.get_logger().info(
            f"[baseline] median={median:.2f}N peak={peak:.2f}N "
            f"n_samples={self._wrench_smoother.n_in_window(now, 0.2)}"
        )
        return median

    def _send_pose(self, move_robot, pose, stiffness=None, damping=None):
        """Publish pose via set_pose_target, with optional DART noise.

        When CHEATCODE_NOISE_POS > 0, adds Gaussian noise to translation.
        When CHEATCODE_NOISE_ROT > 0, composes a small-angle axis-angle
        rotation onto the orientation quaternion. The ORIGINAL clean pose
        object is mutated in place (no copy), which is fine because
        _calc_pose builds a fresh Pose every call.

        stiffness/damping forward through to set_pose_target so the higher
        align/descend stiffness profile is preserved through the wrapper.
        """
        if self._noise_pos > 0:
            pose.position.x += float(self._rng.normal(0, self._noise_pos))
            pose.position.y += float(self._rng.normal(0, self._noise_pos))
            pose.position.z += float(self._rng.normal(0, self._noise_pos))
        if self._noise_rot > 0:
            axis = self._rng.normal(size=3)
            n = float(np.linalg.norm(axis))
            if n > 1e-9:
                axis /= n
                angle = float(self._rng.normal(0, self._noise_rot))
                half = 0.5 * angle
                s = math.sin(half)
                q_noise = (math.cos(half), axis[0]*s, axis[1]*s, axis[2]*s)
                q_cur = (pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z)
                qn = quaternion_multiply(q_noise, q_cur)
                pose.orientation.w, pose.orientation.x = float(qn[0]), float(qn[1])
                pose.orientation.y, pose.orientation.z = float(qn[2]), float(qn[3])
        if stiffness is None and damping is None and self._high_stiffness:
            stiffness = [400.0, 400.0, 400.0, 150.0, 150.0, 150.0]
            damping = [200.0, 200.0, 200.0, 60.0, 60.0, 60.0]
        if stiffness is not None and damping is not None:
            self.set_pose_target(
                move_robot=move_robot,
                pose=pose,
                stiffness=stiffness,
                damping=damping,
            )
        else:
            self.set_pose_target(move_robot=move_robot, pose=pose)

    def _calc_pose(self, port, plug, grip, z_offset,
                   slerp_frac=1.0, pos_frac=1.0, reset_integrator=False,
                   xy_offset=(0.0, 0.0),
                   rot_bias_quat=(1.0, 0.0, 0.0, 0.0)):
        q_port = self._q(port.rotation)
        q_plug = self._q(plug.rotation)
        q_grip = self._q(grip.rotation)

        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        q_target = quaternion_multiply(q_diff, q_grip)
        # Apply small-angle rotation bias to the target if non-identity.
        # Used by the approach/aligning/settling phases to land the cable
        # at a rotated handoff state; the re-align loop in descending
        # passes identity to drive orientation back to true port-aligned.
        if rot_bias_quat[0] < 1.0 - 1e-9:
            q_target = quaternion_multiply(rot_bias_quat, q_target)
        q_blend = quaternion_slerp(q_grip, q_target, slerp_frac)

        gx, gy, gz = grip.translation.x, grip.translation.y, grip.translation.z
        gz_off = gz - plug.translation.z

        # The xy_offset shifts the *convergence target*, not the final command.
        # Adding it after the I-term made the integrator chase port, dragging
        # the plug back toward the unbiased port within ~10 iters and washing
        # out the offset before the descent could bind. Folding it into the
        # PI reference makes the offset truly sticky.
        ref_x = port.translation.x + xy_offset[0]
        ref_y = port.translation.y + xy_offset[1]

        ex = ref_x - plug.translation.x
        ey = ref_y - plug.translation.y

        if reset_integrator:
            self._ix = 0.0
            self._iy = 0.0
        else:
            self._ix = np.clip(self._ix + ex, -self._max_windup, self._max_windup)
            self._iy = np.clip(self._iy + ey, -self._max_windup, self._max_windup)

        tx = ref_x + self._i_gain * self._ix
        ty = ref_y + self._i_gain * self._iy
        tz = port.translation.z + z_offset + gz_off

        bx = pos_frac * tx + (1.0 - pos_frac) * gx
        by = pos_frac * ty + (1.0 - pos_frac) * gy
        bz = pos_frac * tz + (1.0 - pos_frac) * gz

        return Pose(
            position=Point(x=bx, y=by, z=bz),
            orientation=Quaternion(w=q_blend[0], x=q_blend[1], y=q_blend[2], z=q_blend[3]),
        )

    def _approach(self, move_robot, send_feedback):
        send_feedback("Approaching port...")
        # Per-task knobs. approach_converge_frac > 0 turns on Stage 2 (PI
        # lock-in at the end of approach) so _approach() exits with the
        # cable already aligned. SFP keeps it at 0.0 → byte-identical to
        # the prior single-stage cosine-only approach.
        params = CONNECTOR_PARAMS.get(self._task.plug_type, DEFAULT_PARAMS)
        conv_frac = params.get("approach_converge_frac", 0.0)
        ctrl_stiff = params.get("control_stiffness")
        ctrl_damp = params.get("control_damping")
        xy_thresh = params.get("align_xy_threshold", 0.0015)
        vel_thresh = params.get("align_plug_vel_threshold", 0.0003)

        # Diagnostic-only: log gripper-to-port distance at trial start.
        # We previously had a bad_start threshold here, but the heuristic
        # was unreliable (false positives from cable swing). Dataset
        # filtering relies on tier3 score >= 75 instead.
        port0 = self._port_tf()
        grip0 = self._grip_tf()
        d_m = 0.20  # fallback if TF lookup fails
        if port0 and grip0:
            dx = grip0.translation.x - port0.translation.x
            dy = grip0.translation.y - port0.translation.y
            dz = grip0.translation.z - port0.translation.z
            d_m = math.sqrt(dx*dx + dy*dy + dz*dz)
            d_mm = d_m * 1000
            self._publish_event(
                f"approach_start:gripper_to_port={d_mm:.0f}mm"
            )

        # Trapezoidal velocity profile (ported from V5 ImprovedCheatCode):
        # linear ramp-up (RAMP_S) → cruise at PEAK_VEL_MS → linear ramp-down.
        # Replaces the prior cosine profile which spent ~30 % of frames in
        # low-velocity tails — those low-Δ frames biased ACT BC training
        # toward "stay still" predictions at eval. Trapezoid keeps the
        # middle ~80 % of frames at cruise velocity, giving cleaner large-Δ
        # action labels. PEAK 40 mm/s + 0.5 s ramps also shaves total
        # approach time roughly in half vs the prior 30 mm/s cosine.
        PEAK_VEL_MS = 0.040  # 40 mm/s peak
        RAMP_S      = 0.5     # 0.5 s linear accel/decel each end
        ramp_iters  = max(1, int(RAMP_S * self._control_hz))
        ramp_dist   = 0.5 * PEAK_VEL_MS * RAMP_S
        if d_m >= 2 * ramp_dist:
            cruise_dist  = d_m - 2 * ramp_dist
            cruise_iters = max(1, int(cruise_dist / PEAK_VEL_MS * self._control_hz))
        else:
            # Short approach — can't reach peak. Symmetric triangle profile,
            # truncate peak velocity to fit.
            cruise_dist  = 0.0
            cruise_iters = 0
            ramp_iters   = max(1, int((d_m / PEAK_VEL_MS) * self._control_hz))
            ramp_dist    = d_m / 2
        transit_iters = 2 * ramp_iters + cruise_iters
        accel = PEAK_VEL_MS / max(1e-6, RAMP_S)  # m/s² during ramp
        # Stage 1 (trapezoid transit) runs for transit_iters; Stage 2 (PI
        # lock-in) adds a per-task PI budget on top. SFP has conv_frac=0,
        # so pi_cap=0 and APPROACH_ITERS == transit_iters → byte-equivalent
        # to V5's pure-trapezoid approach. SC has conv_frac=0.4 → pi_cap is
        # 2/3 of transit_iters, giving ~5–6 s of PI alignment after transit.
        if conv_frac > 0.0:
            pi_cap = int(conv_frac / (1.0 - conv_frac) * transit_iters)
        else:
            pi_cap = 0
        APPROACH_ITERS = transit_iters + pi_cap
        stage2_t       = transit_iters
        self.get_logger().info(
            f"approach: d={d_m*1000:.0f}mm  iters={APPROACH_ITERS}  "
            f"stage2_at={stage2_t}  "
            f"({APPROACH_ITERS / self._control_hz:.1f}s @ {self._control_hz:.0f}Hz)  "
            f"trapezoid peak={PEAK_VEL_MS*1000:.0f}mm/s ramp={RAMP_S:.1f}s"
        )

        stable_count = 0
        prev_plug_xy = None
        stage2_logged = False
        final_xy_err = None
        conv_iters = 0
        for t in range(APPROACH_ITERS):
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if port is None or plug is None or grip is None:
                self.sleep_for(0.03)
                continue

            if t < stage2_t:
                # ── Stage 1: trapezoid blend, integrator off, default stiffness ──
                # 0.08 m hover (raised from 0.05): gives the gripper margin so
                # its body clears the NIC card top during transit. XY/rot
                # biases shift the target so the cable lands at a handoff
                # state matching the approach-policy distribution at eval.
                tp = t / self._control_hz
                if t < ramp_iters:                                  # accel
                    pos = 0.5 * accel * tp * tp
                elif t < ramp_iters + cruise_iters:                  # cruise
                    tp_c = (t - ramp_iters) / self._control_hz
                    pos = ramp_dist + PEAK_VEL_MS * tp_c
                else:                                                # decel
                    tp_d = (t - ramp_iters - cruise_iters) / self._control_hz
                    pos = (ramp_dist + cruise_dist
                           + PEAK_VEL_MS * tp_d - 0.5 * accel * tp_d * tp_d)
                frac = max(0.0, min(1.0, pos / max(1e-6, d_m)))
                pose = self._calc_pose(port, plug, grip, 0.08,
                                       slerp_frac=frac, pos_frac=frac, reset_integrator=True,
                                       xy_offset=self._approach_xy_bias,
                                       rot_bias_quat=self._approach_rot_bias)
                self._send_pose(move_robot, pose)
            else:
                # ── Stage 2: PI lock-in, integrator accumulates, high stiffness ──
                # Integrator starts at 0 (Stage 1 reset every iter). At
                # _i_gain=0.15 and _max_windup=0.05 m it saturates within
                # ~10 iters at typical 5–15 mm offset, commanding the gripper
                # laterally enough to drag the cable through swing. With the
                # 400/200 stiffness profile from CONNECTOR_PARAMS the gripper
                # actually moves; with the prior 90/50 default it would not.
                if not stage2_logged:
                    bx, by = self._approach_xy_bias
                    ex0 = port.translation.x + bx - plug.translation.x
                    ey0 = port.translation.y + by - plug.translation.y
                    xy0 = math.sqrt(ex0 * ex0 + ey0 * ey0)
                    self.get_logger().info(
                        f"[approach_stage2] entry xy_err={xy0*1000:.2f}mm at iter {t}"
                    )
                    stage2_logged = True
                pose = self._calc_pose(port, plug, grip, 0.08,
                                       slerp_frac=1.0, pos_frac=1.0, reset_integrator=False,
                                       xy_offset=self._approach_xy_bias,
                                       rot_bias_quat=self._approach_rot_bias)
                self._send_pose(move_robot, pose,
                                stiffness=ctrl_stiff, damping=ctrl_damp)
                # V1-style stability gate: xy_err and plug_vel both below
                # threshold for 3 consecutive iters → break early.
                bx, by = self._approach_xy_bias
                ex = port.translation.x + bx - plug.translation.x
                ey = port.translation.y + by - plug.translation.y
                xy_err = math.sqrt(ex * ex + ey * ey)
                if prev_plug_xy is not None:
                    dxp = plug.translation.x - prev_plug_xy[0]
                    dyp = plug.translation.y - prev_plug_xy[1]
                    plug_vel = math.sqrt(dxp * dxp + dyp * dyp)
                else:
                    plug_vel = 1.0
                prev_plug_xy = (plug.translation.x, plug.translation.y)
                conv_iters += 1
                final_xy_err = xy_err
                if xy_err < xy_thresh and plug_vel < vel_thresh:
                    stable_count += 1
                    if stable_count >= 3:
                        self.get_logger().info(
                            f"[approach_converged] xy_err={xy_err*1000:.2f}mm "
                            f"after {conv_iters} PI iters"
                        )
                        self.sleep_for(self._control_dt)
                        break
                else:
                    stable_count = 0
            self.sleep_for(self._control_dt)

        if conv_frac > 0.0 and final_xy_err is not None:
            if final_xy_err >= xy_thresh and stable_count < 3:
                self.get_logger().warn(
                    f"[approach_done] capped at {conv_iters} PI iters, "
                    f"final xy_err={final_xy_err*1000:.2f}mm "
                    f"(threshold {xy_thresh*1000:.2f}mm) — descending anyway"
                )
            else:
                self.get_logger().info(
                    f"[approach_done] xy_err={final_xy_err*1000:.2f}mm "
                    f"conv_iters={conv_iters}"
                )

        self.get_logger().info("Approach done, settling...")
        # Removed the prior sleep_for(0.8) — the trapezoid's 0.5 s decel tail
        # already brings the gripper to ~0 velocity before settle starts; the
        # explicit pause was contributing only "stay still" frames to the
        # training labels.

    def _align_and_descend(self, move_robot, get_obs, send_feedback):
        """Settle (sustained PI hold) → descent (straight down + directed correction on backoff).

        Simplified Exp-27 design — see § Experiment 27 in
        docs/experiments/yosub-act-experiments.md. Removes the explicit
        aligning + re-align phases that existed for the (now-abandoned)
        hierarchical-specialist plan. The settle pause's sustained PI
        does the alignment work via integrator action; descent starts
        straight down from the settled position and uses directed
        correction (not random) on backoff.
        """
        params = CONNECTOR_PARAMS.get(self._task.plug_type, DEFAULT_PARAMS)
        # Per-task control profile. SC requires V1's high-stiffness 400/200
        # for PI integrator authority through the alignment sleeve; SFP
        # keeps None so _send_pose falls through to the 90/50 default.
        ctrl_stiff = params.get("control_stiffness")
        ctrl_damp = params.get("control_damping")
        settle_iters = params.get("settle_iters", 20)
        self._ix = 0.0
        self._iy = 0.0

        baseline = self._measure_baseline(get_obs)
        self.get_logger().info(f"Force baseline: {baseline:.1f}N")

        # 0.08 m hover above port. Raised from 0.05 to give gripper margin
        # against the NIC card top.
        z_offset = 0.08
        bias_x, bias_y = self._approach_xy_bias

        # Log handoff state from approach
        _port_start = self._port_tf()
        _plug_start = self._plug_tf()
        if _port_start and _plug_start:
            _ex_s = _port_start.translation.x - _plug_start.translation.x
            _ey_s = _port_start.translation.y - _plug_start.translation.y
            _xy_s_mm = math.sqrt(_ex_s**2 + _ey_s**2) * 1000
            self.get_logger().info(
                f"[settle_start] xy_to_port={_xy_s_mm:.2f}mm  "
                f"bias=({bias_x*1000:+.1f},{bias_y*1000:+.1f})mm"
            )

        # ── ALIGN phase (optional, per-task) ────────────────────────────
        # V1-style stability gate: hold the hover pose and run the existing
        # PI control until xy error < threshold AND plug lateral velocity
        # < threshold for N consecutive iterations, or until align_max_iters.
        # Skipped entirely when align_after_approach=False (SFP).
        #
        # Why this exists: SC has a tight alignment sleeve (~fraction of a
        # mm clearance). Starting descent from a settle that hasn't fully
        # converged virtually guarantees lip binding, which then cascades
        # through V2's aggressive backoff recovery. V1's success on SC
        # depended on this gate; V2 deleted it for ACT chunk-length reasons
        # that don't apply when the policy is just running the task.
        #
        # The PI integrator is NOT reset here — it should keep accumulating
        # so it converges to the biased target during this loop (same
        # behavior as the V2 settle that follows).
        # Fallback align loop: only runs when in-approach convergence is
        # disabled (approach_converge_frac == 0.0). With SC now folding the
        # alignment work into _approach() via Stage 2 PI lock-in, this block
        # is dead code for SC and acts only as a guard for any task that
        # opts into align_after_approach without approach_converge_frac.
        if (params.get("align_after_approach", False)
                and params.get("approach_converge_frac", 0.0) == 0.0):
            self._publish_phase("aligning")
            send_feedback("Aligning...")
            align_max_iters = params.get("align_max_iters", 300)
            align_xy_threshold = params.get("align_xy_threshold", 0.0015)
            plug_vel_threshold = params.get("align_plug_vel_threshold", 0.0003)
            required_stable = 3
            stable_count = 0
            aligned = False
            prev_plug_xy = None
            for align_iter in range(align_max_iters):
                port = self._port_tf()
                plug = self._plug_tf()
                grip = self._grip_tf()
                if not all([port, plug, grip]):
                    self.sleep_for(self._control_dt)
                    continue

                # Use BIASED target for the error metric — settle and
                # descent will reference the same biased point, so
                # converging here on biased coords keeps continuity.
                ref_x = port.translation.x + bias_x
                ref_y = port.translation.y + bias_y
                ex = ref_x - plug.translation.x
                ey = ref_y - plug.translation.y
                xy_err = math.sqrt(ex * ex + ey * ey)

                if prev_plug_xy is not None:
                    dx = plug.translation.x - prev_plug_xy[0]
                    dy = plug.translation.y - prev_plug_xy[1]
                    plug_vel = math.sqrt(dx * dx + dy * dy)
                else:
                    plug_vel = 1.0
                prev_plug_xy = (plug.translation.x, plug.translation.y)

                pose = self._calc_pose(port, plug, grip, z_offset,
                                       xy_offset=self._approach_xy_bias,
                                       rot_bias_quat=self._approach_rot_bias)
                self._send_pose(move_robot, pose,
                                stiffness=ctrl_stiff, damping=ctrl_damp)
                self.sleep_for(self._control_dt)

                if align_iter % 30 == 0:
                    self.get_logger().info(
                        f"[align {align_iter}] xy_err={xy_err*1000:.2f}mm "
                        f"plug_vel={plug_vel*1000:.2f}mm/iter "
                        f"stable={stable_count}/{required_stable}"
                    )

                if xy_err < align_xy_threshold and plug_vel < plug_vel_threshold:
                    stable_count += 1
                    if stable_count >= required_stable:
                        aligned = True
                        break
                else:
                    stable_count = 0

            self._publish_event(
                f"align_done:converged={int(aligned)}:iters={align_iter + 1}"
                f":xy_err={xy_err*1000:.2f}mm"
            )
            if not aligned:
                # V1 held here and refused to descend. We proceed anyway:
                # holding wastes trial time and a partially-aligned descent
                # is usually still recoverable via the (gentler) SC backoff.
                self.get_logger().warn(
                    f"Alignment did not converge in {align_max_iters} iters "
                    f"(final xy_err={xy_err*1000:.2f}mm) — descending anyway"
                )
            else:
                self.get_logger().info(
                    f"Aligned in {align_iter + 1} iters (xy_err={xy_err*1000:.2f}mm)"
                )

        # ── SETTLE phase ────────────────────────────────────────────────
        # Brief PI hold at (port_xy + bias). The integrator (not reset
        # between iters) drives convergence to the biased target; ~1.5 s
        # is enough for PI to converge and cable swing to damp out
        # before descent — verified empirically. Earlier (Exp-27) we ran
        # 7.5 s here, but training the ACT specialist on those data
        # produced a "hover-forever" failure: with chunk_size=40 (2 s
        # lookahead), 87 % of hover-state action chunks contained no
        # descent, so the model rationally learned to stay put. Keeping
        # settle ≤ chunk window guarantees every hover-state label has a
        # descent visible within 2 s — i.e., descent is always present
        # in the chunk the model predicts from any settle frame.
        # NOTE: target is port_xy + bias (NOT true center). bias is the
        # per-trial sampled offset that controls the descent-start XY
        # distribution. Combined with DART noise (CHEATCODE_NOISE_POS),
        # the model sees a realistic noisy "hold" pattern.
        self._publish_phase("settling")
        send_feedback("Settling...")
        # Per-task settle duration: 20 iters (~1.0s) for SFP to keep within
        # ACT chunk-window; 40 iters (~1.6s) for SC to match V1's cable-
        # damp time before descent.
        for settle_iter in range(settle_iters):
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if all([port, plug, grip]):
                pose = self._calc_pose(port, plug, grip, z_offset,
                                       xy_offset=self._approach_xy_bias,
                                       rot_bias_quat=self._approach_rot_bias)
                self._send_pose(move_robot, pose,
                                stiffness=ctrl_stiff, damping=ctrl_damp)
            self.sleep_for(self._control_dt)
            if settle_iter == 0 or settle_iter == 17 or settle_iter == 34:
                if plug is not None and port is not None:
                    ex_s = port.translation.x - plug.translation.x
                    ey_s = port.translation.y - plug.translation.y
                    xy_s_mm = math.sqrt(ex_s**2 + ey_s**2) * 1000
                    self.get_logger().info(
                        f"[settle {settle_iter}] xy_to_port={xy_s_mm:.2f}mm  "
                        f"bias=({bias_x*1000:+.1f},{bias_y*1000:+.1f})mm"
                    )

        # Approach-only mode: collect approach + settle but skip descent.
        # Slicing from a full-mode bag produces the same data; this mode
        # is preserved for users who want approach-only bags.
        if self._approach_only:
            self.get_logger().info(
                f"Approach-only mode: settled at "
                f"bias=({bias_x*1000:+.1f},{bias_y*1000:+.1f})mm"
            )
            return z_offset

        # ── DESCEND phase ───────────────────────────────────────────────
        # Critical phase event for the dataset slicer:
        # scripts/phase_to_slice_csv.py uses "settling" event time as the
        # boundary. Approach-side dataset ends at settling; insertion-side
        # starts at settling.
        self._publish_phase("descending")
        send_feedback("Descending...")

        F_ref = params["force_ref"]
        F_soft = params["force_soft"]
        F_limit = params["force_limit"]
        # Sustained drop below baseline (N) that counts as lip-support
        # contact. SC opts in (8 N); SFP omits ⇒ 0.0 = trigger off. See
        # the `force_drop` comment in CONNECTOR_PARAMS["sc"] for why
        # signed-delta needs this companion trigger.
        F_drop = params.get("force_drop", 0.0)
        # Per-task off-center gate for backoff. SC tightens this to
        # 0.8 mm so 1.5-2 mm cable_to_port still counts as off-center
        # (sub-mm sleeve clearance). SFP keeps the default 2 mm.
        BACKOFF_DISTANCE_GATE_M = params.get(
            "backoff_distance_gate_m", 0.002
        )
        wsm = self._wrench_smoother
        win = self._wrench_window_s

        # Descent starts STRAIGHT DOWN from the settled position. The cable
        # is currently at (port_xy + bias) thanks to settle's PI convergence;
        # to keep the descent target lateral-stable we MUST set the descent
        # xy_offset to the SAME bias. Setting it to zero would change the PI
        # reference from (port + bias) to port, and the integrator (which
        # accumulated during settle for the biased target) would suddenly
        # see a large -bias error and pull the cable toward true center
        # before any contact happens — defeating the whole point of the
        # bias-augmented descent.
        # On backoff, this offset is REPLACED with a directed step toward
        # the true port center (cable's current xy + step, expressed as
        # offset from port).
        descent_xy_offset = np.array(self._approach_xy_bias, dtype=float)
        # Don't reset the integrator — it's already converged to the bias
        # point during settle. Reset only happens on backoff (when we
        # change the reference target).
        backoff_count = 0
        consecutive_jams = 0
        # CHEATCODE_DESCENT_OFFSET_XY (was: σ for random resample on
        # backoff) is now repurposed as the noise σ added to the directed
        # correction step, so the model doesn't memorize a fixed direction.
        correction_noise_sigma = self._descent_offset_sigma
        # Cooldown after a backoff: skip backoff trigger for N control
        # iterations so the cable physically responds to the new offset
        # before we reassess. Without this, force readings (which lag the
        # commanded pose by ~200ms of cable physics) keep firing backoff
        # against the OLD position and the offset walks chaotically.
        # Two-phase backoff recovery to fix the lip-stuck failure:
        #   Phase A (lift only): raise z, KEEP xy at current cable position.
        #     Gripper goes straight up; cable comes off the lip into clean
        #     air. No friction during the subsequent lateral correction.
        #   Phase B (lateral retarget): NOW switch xy to (0,0) and reset PI.
        #     Cable, dangling in air, gets pulled to port-center with no
        #     friction resisting.
        # Single-phase recovery (lift + xy-retarget simultaneous) failed
        # because PI's lateral force couldn't drag the cable from xy=10mm
        # to xy=0 while the cable was still in contact with the port lip.
        BACKOFF_PHASE_A_ITERS = 10  # ~0.5s lift before lateral retarget
        BACKOFF_PHASE_B_ITERS = 10  # ~0.5s lateral retarget then resume descent
        BACKOFF_COOLDOWN_ITERS = BACKOFF_PHASE_A_ITERS + BACKOFF_PHASE_B_ITERS
        cooldown_remaining = 0
        # When `cooldown_remaining > BACKOFF_PHASE_B_ITERS` we're in phase A
        # (xy held at pre-backoff cable position). When ≤ phase B, we've
        # transitioned to xy=(0,0).
        backoff_xy_hold = None  # the xy_offset we keep cable at during phase A
        # If cable is more than this far from port, no realistic
        # directed-correction step recovers the trial — abort early
        # rather than burn 8 backoffs chasing a falling cable.
        OFF_CARD_ABORT_M = 0.030  # 30 mm — well past SFP card half-width
        # Hover-stuck detection: after 2 consecutive backoffs where the
        # cable was already centered (xy < 3mm), we conclude the cable
        # is hovering above the port without descending — neither stuck
        # on a lip nor swinging off — just refusing to slip in. Trigger
        # a stronger recovery: lift back to approach altitude (60mm) and
        # hold, then resume descent with a fresh cable state.
        HOVER_STUCK_GATE_M = 0.015   # cable "centered enough" — widened from 8mm to
                                     # catch lip-stuck pattern (cable_to_port at backoff
                                     # time consistently 9-12mm in those failures).
                                     # Hover_recovery's lift+retarget is also the right
                                     # move for off-center-on-lip cable, so widening
                                     # is strictly upside.
        HOVER_STUCK_THRESHOLD = 2    # consecutive centered backoffs to trigger recovery
        # Count-based escalation: if normal backoff hasn't worked in N
        # tries regardless of xy, fall through to the big-lift recovery.
        HOVER_BACKOFF_COUNT_TRIGGER = 4
        HOVER_RECOVERY_LIFT_M = 0.060  # 60 mm — approach-style altitude
        HOVER_RECOVERY_HOLD_ITERS = 40  # ~2s hold to settle cable
        MAX_HOVER_RECOVERIES = 3  # abort trial if recovery keeps firing — prevents runaway loops that drag the engine past its trial timeout
        consecutive_centered_backoffs = 0
        hover_recovery_count = 0  # how many times hover recovery has fired this trial

        # XY-probe before backoff (ported from V5 ImprovedCheatCode):
        # When force exceeds F_soft, first try a small circular XY scan at
        # the current target before falling through to the lift+retarget
        # backoff. "Wiggle to find slot" produces clean deterministic action
        # labels for ACT BC and avoids destroying partial-engagement progress
        # in SC's tight alignment sleeve. Holds z and oscillates xy_offset by
        # PROBE_RADIUS_M for up to PROBE_MAX_ITERS, exiting early on force
        # release (df_med ≤ F_ref). On exhaustion, falls through to backoff.
        PROBE_RADIUS_M     = 0.0015          # 1.5 mm
        PROBE_PERIOD_S     = 0.5              # 2 Hz scan rate
        PROBE_MAX_S        = 1.5              # max probe duration before backoff fallback
        PROBE_PERIOD_ITERS = max(2, int(PROBE_PERIOD_S * self._control_hz))
        PROBE_MAX_ITERS    = int(PROBE_MAX_S * self._control_hz)
        probe_iter         = 0               # 0 = idle; 1..PROBE_MAX_ITERS while active
        probe_center_xy    = None            # snapshot of descent_xy_offset at probe start

        # Wall-clock guards. Task time_limit is 180s; approach already burned
        # ~15-20s. The TF-fail guard bails when /tf delivery stalls for
        # >1s straight (multi-worker Zenoh hiccup, observed empirically),
        # which previously spun this loop forever.
        ALIGN_DESCEND_MAX_S = 140.0
        TF_FAIL_MAX_S       = 1.0
        loop_start_s        = self._now_s()
        tf_fail_start_s     = None

        while True:
            if self._now_s() - loop_start_s > ALIGN_DESCEND_MAX_S:
                self.get_logger().error(
                    f"_align_and_descend timed out after {ALIGN_DESCEND_MAX_S:.0f}s — aborting trial"
                )
                send_feedback("align_descend timeout")
                break
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                now_s = self._now_s()
                if tf_fail_start_s is None:
                    tf_fail_start_s = now_s
                elif now_s - tf_fail_start_s > TF_FAIL_MAX_S:
                    self.get_logger().error(
                        f"TF unavailable for {now_s - tf_fail_start_s:.1f}s — aborting trial"
                    )
                    send_feedback("tf_stall")
                    break
                self.sleep_for(0.03)
                continue
            tf_fail_start_s = None

            base_step = params["coarse_step"] if z_offset > params["fine_threshold"] else params["fine_step"]

            # Cable→port distance (ground truth) — used to gate backoff so
            # we don't retreat when cable is dead-on the port and merely
            # rubbing the lip during good insertion.
            dx_to_port = port.translation.x - plug.translation.x
            dy_to_port = port.translation.y - plug.translation.y
            cable_to_port = math.sqrt(dx_to_port**2 + dy_to_port**2)

            # Pre-step force read. Use median for both taper AND backoff —
            # peak-based backoff fires on every transient spike (one
            # backoff per ~100ms), and the directed correction step then
            # runs against a stale cable position because cable physics
            # lag the commanded pose by ~200ms. Median = sustained
            # pressure only.
            #
            # force_trigger_abs (per-task): SFP uses |f - baseline| because
            # pressing on the card top REDUCES the wrench magnitude (the
            # surface offloads gravity on Fz so vector magnitude drops
            # below baseline). SC has no card-top — magnitude only rises
            # during legitimate seating — so signed delta avoids false
            # backoffs from transient magnitude dips. V1 always used
            # signed delta; this knob restores that behavior on SC.
            now = self._now_s()
            force_med = wsm.median_magnitude(now, window_s=win)
            force_max = wsm.peak_magnitude(now, window_s=win)
            if params.get("force_trigger_abs", True):
                df_med = abs(force_med - baseline)
                df_max = abs(force_max - baseline)
            else:
                df_med = max(0.0, force_med - baseline)
                df_max = max(0.0, force_max - baseline)

            # Backoff fires on sustained median pressure, with a cooldown
            # after each event so the cable physically moves before the
            # next assessment. Cooldown decrement happens regardless of
            # current force reading.
            if cooldown_remaining > 0:
                cooldown_remaining -= 1
                # Phase A → Phase B transition: when cooldown drops below
                # BACKOFF_PHASE_B_ITERS, switch xy target from "hold cable
                # in place" to "(0, 0) port-center" and reset PI. Cable is
                # now in the air (z lifted) so PI can pull it laterally
                # without friction.
                if (backoff_xy_hold is not None
                        and cooldown_remaining == BACKOFF_PHASE_B_ITERS):
                    descent_xy_offset = np.array([0.0, 0.0])
                    if correction_noise_sigma > 0:
                        descent_xy_offset += self._rng.normal(
                            0, correction_noise_sigma, size=2)
                    self._ix = 0.0
                    self._iy = 0.0
                    self._publish_event(
                        f"backoff_phase_b:to_port_center"
                    )
                    backoff_xy_hold = None
            backed_off = False
            # Off-center gate (BACKOFF_DISTANCE_GATE_M, per-task): if
            # cable is right at the port, force feedback is almost
            # certainly port-rim rubbing during good insertion — don't
            # retreat with the lift+retarget backoff. But we still want
            # the *XY probe* to fire there: when cable is centered but
            # force is high (e.g., bottom snagged on lip edge,
            # rotational misalignment), a small XY scan with z held may
            # engage the slot without disturbing the descent target.
            #
            # Force-drop trigger: SC's signed-delta `force_trigger_abs=
            # False` clamps `df_med = max(0, force_med - baseline)` to
            # 0 when the port lip supports cable weight (sustained drop
            # below baseline). The `force_drop_med` companion catches
            # that case so backoff fires on lip contact. SFP keeps
            # F_drop=0.0 so this OR-branch never contributes.
            force_drop_med = max(0.0, baseline - force_med)
            force_high = (
                cooldown_remaining == 0
                and (df_med >= F_soft
                     or (F_drop > 0.0 and force_drop_med >= F_drop))
            )
            high_force_offcenter = (
                force_high
                and cable_to_port > BACKOFF_DISTANCE_GATE_M
            )

            # ── XY-probe gating ──────────────────────────────────────────
            # If we're already probing OR just hit the backoff-trigger
            # condition, run the probe state machine. Probe overrides
            # descent_xy_offset with a circular pattern and holds z. It
            # ends on either force release (df_med ≤ F_ref) or exhaustion
            # (probe_iter ≥ PROBE_MAX_ITERS). On exhaustion control falls
            # through to either backoff (off-center) or tapered descent
            # (centered).
            probe_active = False
            if force_high and probe_iter == 0:
                probe_center_xy = descent_xy_offset.copy()
                probe_iter = 1
                self._publish_event(
                    f"xy_probe_start:r={PROBE_RADIUS_M*1000:.1f}mm"
                    f":T={PROBE_PERIOD_S}s:max={PROBE_MAX_S}s"
                    f":df_med={df_med:.2f}N:to_port={cable_to_port*1000:.2f}mm"
                )
            if probe_iter > 0:
                if df_med <= F_ref:
                    descent_xy_offset = probe_center_xy.copy()
                    self._publish_event(
                        f"xy_probe_end:reason=release"
                        f":iters={probe_iter}:df_med={df_med:.2f}N"
                    )
                    probe_iter = 0
                    probe_center_xy = None
                elif probe_iter >= PROBE_MAX_ITERS:
                    descent_xy_offset = probe_center_xy.copy()
                    self._publish_event(
                        f"xy_probe_end:reason=exhausted"
                        f":iters={probe_iter}:df_med={df_med:.2f}N"
                    )
                    probe_iter = 0
                    probe_center_xy = None
                else:
                    phase = (probe_iter - 1) * (2.0 * math.pi / PROBE_PERIOD_ITERS)
                    descent_xy_offset = probe_center_xy + np.array([
                        PROBE_RADIUS_M * math.cos(phase),
                        PROBE_RADIUS_M * math.sin(phase),
                    ])
                    probe_iter += 1
                    probe_active = True

            if probe_active:
                step = 0.0                          # hold z while probing
            elif high_force_offcenter:
                step = -base_step * 0.5
                backed_off = True
            elif df_med <= F_ref:
                step = base_step
            else:
                scale = 1.0 - (df_med - F_ref) / (F_soft - F_ref)
                step = base_step * max(0.1, scale)

            # Hold z during cooldown so the lift actually has time to take
            # effect. Without this gate, descent ticks during the 1s cooldown
            # would consume ~24mm of altitude — wiping out our 20mm lift —
            # and during hover_recovery's 2s hold would consume ~48mm of the
            # 60mm assigned, dropping the cable right back to the port lip.
            if cooldown_remaining == 0:
                z_offset -= step

            if z_offset < params["max_depth"]:
                self.get_logger().info("Reached max depth")
                break

            # On backoff (force spike), apply DIRECTED correction toward
            # the true port center — informed adjustment instead of random
            # search. The cheatcode has access to /scoring/tf for ground
            # truth at training time; the model only sees the resulting
            # action stream and learns the "bump → step toward port"
            # pattern from those actions.
            # Step size: half the cable→port distance, capped at 5 mm.
            # Plus small Gaussian noise (correction_noise_sigma, default
            # from CHEATCODE_DESCENT_OFFSET_XY env var) so the model
            # doesn't memorize a deterministic step direction.
            if backed_off:
                backoff_count += 1
                old = descent_xy_offset.copy()
                norm = cable_to_port
                # Off-card abort: if cable has drifted far past port, no
                # backoff sequence will recover it (cable physics + PI
                # chase a falling cable). End the descent so the trial
                # fails cleanly instead of running 8 backoffs of garbage.
                if norm > OFF_CARD_ABORT_M:
                    self.get_logger().warn(
                        f"OFF-CARD ABORT: cable_to_port={norm*1000:.1f}mm "
                        f"> {OFF_CARD_ABORT_M*1000:.0f}mm threshold"
                    )
                    self._publish_event(
                        f"off_card_abort:to_port={norm*1000:.2f}mm"
                        f":threshold={OFF_CARD_ABORT_M*1000:.0f}mm"
                    )
                    break

                # Hover-stuck detection: cable is centered (xy<gate) but we
                # keep firing backoffs without descending. Two consecutive
                # centered backoffs → trigger stronger recovery (lift to
                # approach altitude, hold to settle cable, then resume).
                # Also escalate after N total backoffs regardless of xy —
                # by then normal backoff demonstrably isn't working, even
                # if cable is too off-center to satisfy the gate.
                if norm < HOVER_STUCK_GATE_M:
                    consecutive_centered_backoffs += 1
                else:
                    consecutive_centered_backoffs = 0
                hover_trigger_reason = None
                if consecutive_centered_backoffs >= HOVER_STUCK_THRESHOLD:
                    hover_trigger_reason = f"centered({consecutive_centered_backoffs} consec)"
                elif backoff_count >= HOVER_BACKOFF_COUNT_TRIGGER:
                    hover_trigger_reason = f"count({backoff_count} backoffs)"
                if hover_trigger_reason is not None:
                    hover_recovery_count += 1
                    if hover_recovery_count > MAX_HOVER_RECOVERIES:
                        self.get_logger().warn(
                            f"Exceeded {MAX_HOVER_RECOVERIES} hover recoveries, "
                            f"stopping descent"
                        )
                        break
                    self.get_logger().warn(
                        f"HOVER-STUCK: trigger={hover_trigger_reason} "
                        f"to_port={norm*1000:.2f}mm — "
                        f"lifting to {HOVER_RECOVERY_LIFT_M*1000:.0f}mm and resetting"
                    )
                    self._publish_event(
                        f"hover_recovery:N={hover_recovery_count}"
                        f":trigger={hover_trigger_reason}"
                        f":to_port={norm*1000:.2f}mm"
                        f":lift_to={HOVER_RECOVERY_LIFT_M*1000:.0f}mm"
                    )
                    z_offset = HOVER_RECOVERY_LIFT_M
                    descent_xy_offset = np.array([0.0, 0.0])
                    self._ix = 0.0
                    self._iy = 0.0
                    cooldown_remaining = HOVER_RECOVERY_HOLD_ITERS
                    backoff_xy_hold = None  # phase A is implicit during the long hold
                    consecutive_centered_backoffs = 0
                    continue  # skip the normal phase-A setup below
                # Two-phase recovery, see definition above.
                # PHASE A: lift z while keeping xy at the cable's CURRENT
                # offset position. This pulls the cable straight up off
                # the lip, no lateral motion → no friction. We compute
                # cable's current offset-from-port (plug.x - port.x) and
                # set descent_xy_offset to THAT, so the gripper command
                # holds the cable laterally where it is.
                # ADDITIVE lift: previously this was a clamp `z_offset = max(z, 20mm)`,
                # but z_offset was already at or near 20mm by the time a backoff
                # fired (the descent step taper holds it there once force is high),
                # so the clamp was a no-op — cable barely lifted, lip friction
                # persisted, lateral retarget couldn't pull cable to center.
                # Force a real additive lift each backoff. OFF_CARD_ABORT (30mm
                # cable_to_port) and HOVER_RECOVERY (60mm absolute lift on
                # hover-stuck) cap any runaway.
                #
                # Per-task: SFP uses 20 mm — the SFP card-top binding pattern
                # needs a large lift to clear lip friction so the lateral
                # retarget can drag the cable through air. SC seats through a
                # tight alignment sleeve; a 20 mm lift destroys partial-
                # engagement progress and forces the descent to re-do all the
                # alignment work. 5 mm is enough to break lip contact without
                # losing engagement.
                BACKOFF_LIFT_M = params.get("backoff_lift_m", 0.020)
                BACKOFF_LIFT_CAP_M = 0.060  # 60mm cap — prevents unbounded climb
                z_offset = min(z_offset + BACKOFF_LIFT_M, BACKOFF_LIFT_CAP_M)
                # Phase A target: where the cable currently is (so PI
                # doesn't try to drag it laterally yet).
                backoff_xy_hold = np.array([
                    plug.translation.x - port.translation.x,
                    plug.translation.y - port.translation.y,
                ], dtype=float)
                descent_xy_offset = backoff_xy_hold.copy()
                # Reset PI integrator so the new offset reference takes
                # effect cleanly (old integrator was building against a
                # different target).
                self._ix = 0.0
                self._iy = 0.0
                cooldown_remaining = BACKOFF_COOLDOWN_ITERS
                self.get_logger().info(
                    f"[backoff {backoff_count}] z={z_offset:.4f} df_med={df_med:.1f}N "
                    f"cable_to_port={norm*1000:.2f}mm — target port-center "
                    f"old_offset={old[0]*1000:+.2f},{old[1]*1000:+.2f} -> "
                    f"{descent_xy_offset[0]*1000:+.2f},{descent_xy_offset[1]*1000:+.2f} mm "
                    f"(cooldown {BACKOFF_COOLDOWN_ITERS})"
                )
                self._publish_event(
                    f"backoff:N={backoff_count}:z={z_offset*1000:.2f}mm"
                    f":df_med={df_med:.2f}N:to_port={norm*1000:.2f}mm"
                    f":old_offset={old[0]*1000:.2f},{old[1]*1000:.2f}mm"
                    f":new_offset={descent_xy_offset[0]*1000:.2f},{descent_xy_offset[1]*1000:.2f}mm"
                )
                if backoff_count >= MAX_DESCENT_BACKOFF_ATTEMPTS:
                    self.get_logger().warn(
                        f"Exceeded {MAX_DESCENT_BACKOFF_ATTEMPTS} backoff attempts, "
                        f"stopping descent"
                    )
                    break

            plug_z_before = plug.translation.z

            pose = self._calc_pose(port, plug, grip, z_offset,
                                   xy_offset=tuple(descent_xy_offset))
            # Per-task control profile (params["control_stiffness"]).
            # SFP uses None → set_pose_target's 90/50 default. SC uses
            # 400/200 to give PI authority through the alignment sleeve.
            self._send_pose(move_robot, pose,
                            stiffness=ctrl_stiff, damping=ctrl_damp)

            # Single sleep — wrench buffer keeps filling in the background
            # at the topic's native rate (no extra polls, no extra sleeps).
            self.sleep_for(self._control_dt)

            # Post-step read for jam check + logging.
            now_post = self._now_s()
            force_post_med = wsm.median_magnitude(now_post, window_s=win)
            force_post_max = wsm.peak_magnitude(now_post, window_s=win)
            df = force_post_med - baseline
            if params.get("force_trigger_abs", True):
                df_jam = force_post_max - baseline
                jam_trigger = abs(df_jam) > F_limit
            else:
                df_jam = max(0.0, force_post_max - baseline)
                jam_trigger = df_jam > F_limit

            plug_after = self._plug_tf()
            dz = (plug_z_before - plug_after.translation.z) if plug_after else step

            # Jam check uses peak — never miss a spike. Trigger semantics
            # (abs vs signed) follow the per-task force_trigger_abs knob:
            # SFP needs abs to catch card-top press (magnitude drops below
            # baseline); SC uses signed delta to avoid false jams on
            # transient magnitude dips.
            if jam_trigger:
                if dz < base_step * 0.3:
                    consecutive_jams += 1
                    self.get_logger().warn(
                        f"JAM: df={df:.1f}N df_max={df_jam:.1f}N "
                        f"dz={dz*1000:.3f}mm z={z_offset:.4f} jams={consecutive_jams}"
                    )
                    self._publish_event(
                        f"jam:N={consecutive_jams}:z={z_offset*1000:.2f}mm"
                        f":df={df:.2f}N:df_jam={df_jam:.2f}N:dz={dz*1000:.3f}mm"
                    )
                    z_offset += base_step * 3
                    pose = self._calc_pose(port, plug, grip, z_offset,
                                           xy_offset=tuple(descent_xy_offset))
                    self._send_pose(move_robot, pose)
                    self.sleep_for(0.1)
                    if consecutive_jams >= 5:
                        self.get_logger().warn("Too many jams, stopping")
                        break
                else:
                    self.get_logger().info(
                        f"Force df={df:.1f}N (max={df_jam:.1f}N) but moving dz={dz*1000:.3f}mm"
                    )
                    consecutive_jams = 0
            else:
                if consecutive_jams > 0 and abs(df_jam) < F_limit * 0.3:
                    consecutive_jams = max(0, consecutive_jams - 1)

            if z_offset % 0.005 < base_step:
                if plug_after is not None:
                    ex_now = port.translation.x - plug_after.translation.x
                    ey_now = port.translation.y - plug_after.translation.y
                    xy_now_mm = math.sqrt(ex_now**2 + ey_now**2) * 1000
                    zgap_now_mm = (plug_after.translation.z - port.translation.z) * 1000
                else:
                    xy_now_mm = -1.0
                    zgap_now_mm = -1.0
                grip_now = self._grip_tf()
                if grip_now is not None and plug_after is not None:
                    grip_plug_dz_mm = (grip_now.translation.z - plug_after.translation.z) * 1000
                else:
                    grip_plug_dz_mm = -1.0
                # fz_now: latest single z-component for human inspection only.
                latest = wsm.latest()
                fz_now = latest[2] if latest else 0.0
                n_samp = wsm.n_in_window(now_post, win)
                self.get_logger().info(
                    f"[desc] z={z_offset:.4f} df={df:.1f}N df_max={df_jam:.1f}N "
                    f"fz={fz_now:.1f}N n={n_samp} dz={dz*1000:.3f}mm "
                    f"xy={xy_now_mm:.2f}mm zgap={zgap_now_mm:.2f}mm "
                    f"gpz={grip_plug_dz_mm:.2f}mm step={step*1000:.3f}mm"
                )

        plug = self._plug_tf()
        port = self._port_tf()
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            xy_mm = math.sqrt(ex**2+ey**2)*1000
            z_gap = plug.translation.z - port.translation.z
            self.get_logger().info(
                f"Done: xy={xy_mm:.2f}mm "
                f"z_gap={z_gap*1000:.2f}mm backoff_count={backoff_count}"
            )

        # Stash backoff_count for the post-stabilization descent_done event
        # in insert_cable — the cable doesn't reach its physical resting
        # position until after the 1s stabilizing loop drives gripper to
        # final_z. Capturing here would report mid-fall position.
        self._last_backoff_count = backoff_count
        return z_offset

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        mode = "APPROACH-ONLY (no insertion)" if self._approach_only else "FULL (approach + insert)"
        self.get_logger().info(f"ImprovedCheatCode.insert_cable() mode={mode} task: {task}")
        self._task = task
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        self._ix = 0.0
        self._iy = 0.0

        for frame in [self._port_frame, self._plug_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' unavailable, aborting")
                return False

        self._sample_approach_biases()
        self._publish_phase("approach")
        self._approach(move_robot, send_feedback)
        final_z = self._align_and_descend(move_robot, get_observation, send_feedback)
        self._publish_phase("done")

        self.get_logger().info("Stabilizing...")
        for _ in range(20):
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if all([port, plug, grip]):
                pose = self._calc_pose(port, plug, grip, final_z)
                self._send_pose(move_robot, pose)
            self.sleep_for(self._control_dt)

        # Publish descent_done AFTER the stabilization loop so the event
        # records the cable's physically-settled position, not its mid-fall
        # location at the moment the descent loop terminated. The 20-iter
        # stabilizing loop drives the gripper PI to converge on final_z;
        # cable plug catches up over those ~1s.
        plug = self._plug_tf()
        plug_age = getattr(self, '_last_tf_age_s', -1.0)
        port = self._port_tf()
        port_age = getattr(self, '_last_tf_age_s', -1.0)
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            xy_mm = math.sqrt(ex**2+ey**2)*1000
            z_gap = (plug.translation.z - port.translation.z) * 1000
            bc = getattr(self, '_last_backoff_count', 0)
            # tf_age = max(plug, port) age — flags a stale cable→port read.
            tf_age_ms = max(plug_age, port_age) * 1000 if max(plug_age, port_age) >= 0 else -1
            self._publish_event(
                f"descent_done:xy={xy_mm:.2f}mm:z_gap={z_gap:.2f}mm"
                f":backoff_count={bc}:tf_age={tf_age_ms:.0f}ms"
            )

        # End-of-trial joint-space hold: command the controller to freeze the
        # arm at its current joint configuration. Goal: prevent drift in the
        # window after on_deactivate() but before the next trial's joint reset
        # fires. Drift in this window lets the new cable attach to a displaced
        # gripper, producing pathological grasps in the next trial.
        # Joint-space command — DART pose noise is intentionally NOT applied.
        obs = get_observation()
        if obs is not None and obs.joint_states is not None and len(obs.joint_states.position) >= 6:
            current_joints = list(obs.joint_states.position[:6])
            self.get_logger().info(
                f"[end_hold] freezing arm at joints=[{', '.join(f'{j:.3f}' for j in current_joints)}]"
            )
            hold_cmd = JointMotionUpdate(
                target_state=JointTrajectoryPoint(positions=current_joints),
                target_stiffness=[500.0, 500.0, 500.0, 200.0, 200.0, 200.0],
                target_damping=[40.0, 40.0, 40.0, 15.0, 15.0, 15.0],
                trajectory_generation_mode=TrajectoryGenerationMode(
                    mode=TrajectoryGenerationMode.MODE_POSITION,
                ),
            )
            for _ in range(15):
                move_robot(joint_motion_update=hold_cmd)
                self.sleep_for(self._control_dt)
        else:
            self.get_logger().warn("[end_hold] observation unavailable, skipping")

        self.get_logger().info("ImprovedCheatCode.insert_cable() exiting...")
        return True