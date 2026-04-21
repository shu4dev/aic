import math
import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

QuaternionTuple = tuple[float, float, float, float]

CONNECTOR_PARAMS = {
    "sfp": {"max_depth": -0.022, "coarse_step": 0.0008, "fine_step": 0.0002,
            "fine_threshold": 0.01, "force_limit": 15.0},
    "sc":  {"max_depth": -0.020, "coarse_step": 0.0007, "fine_step": 0.0002,
            "fine_threshold": 0.008, "force_limit": 12.0},
}
DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]

APPROACH_HEIGHT = 0.03
APPROACH_STEPS = 80
APPROACH_DT = 0.04
EMA_ALPHA = 0.3
SETTLE_TIME = 1.5


def min_jerk(t):
    t = np.clip(t, 0.0, 1.0)
    return 10 * t**3 - 15 * t**4 + 6 * t**5


class EMAFilter:
    def __init__(self, alpha=EMA_ALPHA):
        self._alpha = alpha
        self._state = None

    def reset(self):
        self._state = None

    def update(self, value):
        if self._state is None:
            self._state = np.array(value, dtype=float)
        else:
            self._state = self._alpha * np.array(value) + (1.0 - self._alpha) * self._state
        return self._state.copy()


class ImprovedCheatCode(Policy):
    def __init__(self, parent_node):
        self._task = None
        self._ix = 0.0
        self._iy = 0.0
        self._max_windup = 0.05
        self._i_gain = 0.15
        self._pos_filter = EMAFilter()
        super().__init__(parent_node)

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

    def _lookup(self, target, source):
        try:
            return self._parent_node._tf_buffer.lookup_transform(target, source, Time()).transform
        except TransformException:
            return None

    def _port_tf(self):
        return self._lookup("base_link", self._port_frame)

    def _plug_tf(self):
        return self._lookup("base_link", self._plug_frame)

    def _grip_tf(self):
        return self._lookup("base_link", "gripper/tcp")

    def _q(self, rot):
        return (rot.w, rot.x, rot.y, rot.z)

    def _get_force(self, get_obs):
        obs = get_obs()
        if obs is None or obs.wrist_wrench is None:
            return 0.0
        f = obs.wrist_wrench.wrench.force
        return math.sqrt(f.x**2 + f.y**2 + f.z**2)

    def _measure_baseline(self, get_obs, n=10):
        samples = []
        for _ in range(n):
            samples.append(self._get_force(get_obs))
            self.sleep_for(0.02)
        return sum(samples) / len(samples)

    def _calc_raw_target(self, port, plug, grip, z_offset,
                         slerp_frac=1.0, pos_frac=1.0, reset_integrator=False):
        q_port = self._q(port.rotation)
        q_plug = self._q(plug.rotation)
        q_grip = self._q(grip.rotation)

        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        q_target = quaternion_multiply(q_diff, q_grip)
        q_blend = quaternion_slerp(q_grip, q_target, slerp_frac)

        gx, gy, gz = grip.translation.x, grip.translation.y, grip.translation.z
        gz_off = gz - plug.translation.z

        ex = port.translation.x - plug.translation.x
        ey = port.translation.y - plug.translation.y

        if reset_integrator:
            self._ix = 0.0
            self._iy = 0.0
        else:
            self._ix = np.clip(self._ix + ex, -self._max_windup, self._max_windup)
            self._iy = np.clip(self._iy + ey, -self._max_windup, self._max_windup)

        tx = port.translation.x + self._i_gain * self._ix
        ty = port.translation.y + self._i_gain * self._iy
        tz = port.translation.z + z_offset + gz_off

        bx = pos_frac * tx + (1.0 - pos_frac) * gx
        by = pos_frac * ty + (1.0 - pos_frac) * gy
        bz = pos_frac * tz + (1.0 - pos_frac) * gz

        return (bx, by, bz), q_blend

    def _send_smoothed_pose(self, move_robot, raw_pos, q_blend):
        smoothed = self._pos_filter.update(raw_pos)
        pose = Pose(
            position=Point(x=float(smoothed[0]), y=float(smoothed[1]), z=float(smoothed[2])),
            orientation=Quaternion(w=q_blend[0], x=q_blend[1], y=q_blend[2], z=q_blend[3]),
        )
        self.set_pose_target(move_robot=move_robot, pose=pose)

    def _approach(self, move_robot, send_feedback):
        send_feedback("Approaching port...")
        self._pos_filter.reset()

        for t in range(APPROACH_STEPS):
            frac = min_jerk(t / APPROACH_STEPS)
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(APPROACH_DT)
                continue
            raw_pos, q_blend = self._calc_raw_target(
                port, plug, grip, APPROACH_HEIGHT,
                slerp_frac=frac, pos_frac=frac, reset_integrator=True)
            self._send_smoothed_pose(move_robot, raw_pos, q_blend)
            self.sleep_for(APPROACH_DT)

        self.get_logger().info("Approach done, settling...")
        self.sleep_for(0.5)

    def _align_and_descend(self, move_robot, get_obs, send_feedback):
        params = CONNECTOR_PARAMS.get(self._task.plug_type, DEFAULT_PARAMS)
        self._ix = 0.0
        self._iy = 0.0
        self._pos_filter.reset()

        baseline = self._measure_baseline(get_obs)
        self.get_logger().info(f"Force baseline: {baseline:.1f}N")

        z_offset = APPROACH_HEIGHT
        consecutive_jams = 0

        send_feedback("Aligning and descending...")

        while True:
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(0.03)
                continue

            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            xy_err = math.sqrt(ex**2 + ey**2)

            if z_offset > params["fine_threshold"]:
                step = params["coarse_step"]
                if xy_err > 0.003:
                    step *= 0.3
            else:
                step = params["fine_step"]

            z_offset -= step

            if z_offset < params["max_depth"]:
                self.get_logger().info("Reached max depth")
                break

            plug_z_before = plug.translation.z

            raw_pos, q_blend = self._calc_raw_target(port, plug, grip, z_offset)
            self._send_smoothed_pose(move_robot, raw_pos, q_blend)
            self.sleep_for(0.035)

            force = self._get_force(get_obs)
            df = force - baseline

            plug_after = self._plug_tf()
            dz = (plug_z_before - plug_after.translation.z) if plug_after else step

            if df > params["force_limit"]:
                if dz < step * 0.3:
                    consecutive_jams += 1
                    self.get_logger().warn(
                        f"JAM: df={df:.1f}N dz={dz*1000:.3f}mm z={z_offset:.4f} jams={consecutive_jams}"
                    )
                    z_offset += step * 3
                    raw_pos, q_blend = self._calc_raw_target(port, plug, grip, z_offset)
                    self._send_smoothed_pose(move_robot, raw_pos, q_blend)
                    self.sleep_for(0.1)
                    if consecutive_jams >= 5:
                        self.get_logger().warn("Too many jams, stopping")
                        break
                else:
                    self.get_logger().info(f"Force df={df:.1f}N but moving dz={dz*1000:.3f}mm")
                    consecutive_jams = 0
            else:
                if consecutive_jams > 0 and df < params["force_limit"] * 0.3:
                    consecutive_jams = max(0, consecutive_jams - 1)

            if z_offset % 0.005 < step:
                self.get_logger().info(
                    f"[desc] z={z_offset:.4f} df={df:.1f}N xy={xy_err*1000:.1f}mm"
                )

        plug = self._plug_tf()
        port = self._port_tf()
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            z_gap = plug.translation.z - port.translation.z
            self.get_logger().info(f"Done: xy={math.sqrt(ex**2+ey**2)*1000:.2f}mm z_gap={z_gap*1000:.2f}mm")

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        self.get_logger().info(f"ImprovedCheatCodeV2.insert_cable() task: {task}")
        self._task = task
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        self._ix = 0.0
        self._iy = 0.0

        for frame in [self._port_frame, self._plug_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' unavailable, aborting")
                return False

        self._approach(move_robot, send_feedback)
        self._align_and_descend(move_robot, get_observation, send_feedback)

        self.get_logger().info("Stabilizing...")
        self.sleep_for(SETTLE_TIME)

        self.get_logger().info("ImprovedCheatCodeV2.insert_cable() exiting...")
        return True