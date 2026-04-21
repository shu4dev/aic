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
    "sfp": {"max_depth": -0.022, "force_limit": 15.0},
    "sc":  {"max_depth": -0.020, "force_limit": 12.0},
}
DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]

APPROACH_HEIGHT = 0.05
APPROACH_TIME = 3.0
APPROACH_SETTLE = 0.3
DESCENT_TIME = 2.0
DT = 0.02
CORR_ALPHA = 0.08
MIN_STEP = 0.00005
SETTLE_TIME = 0.3


def min_jerk(t):
    t = np.clip(t, 0.0, 1.0)
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def min_jerk_vel(t):
    t = np.clip(t, 0.0, 1.0)
    return 30 * t**2 - 60 * t**3 + 30 * t**4


class EMAScalar:
    def __init__(self, alpha):
        self._alpha = alpha
        self._val = 0.0
        self._init = False

    def reset(self):
        self._val = 0.0
        self._init = False

    def update(self, x):
        if not self._init:
            self._val = x
            self._init = True
        else:
            self._val = self._alpha * x + (1.0 - self._alpha) * self._val
        return self._val


class ImprovedCheatCode(Policy):
    def __init__(self, parent_node):
        self._task = None
        self._ix = 0.0
        self._iy = 0.0
        self._max_windup = 0.05
        self._i_gain = 0.15
        self._cx_ema = EMAScalar(CORR_ALPHA)
        self._cy_ema = EMAScalar(CORR_ALPHA)
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

    def _measure_baseline(self, get_obs, n=5):
        samples = []
        for _ in range(n):
            samples.append(self._get_force(get_obs))
            self.sleep_for(0.015)
        return sum(samples) / len(samples)

    def _compute_pose(self, port, plug, grip, z_offset,
                      slerp_frac=1.0, pos_frac=1.0,
                      use_correction=False):
        q_port = self._q(port.rotation)
        q_plug = self._q(plug.rotation)
        q_grip = self._q(grip.rotation)

        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        q_target = quaternion_multiply(q_diff, q_grip)
        q_blend = quaternion_slerp(q_grip, q_target, slerp_frac)

        gx, gy, gz = grip.translation.x, grip.translation.y, grip.translation.z
        gx_off = gx - plug.translation.x
        gy_off = gy - plug.translation.y
        gz_off = gz - plug.translation.z

        base_x = port.translation.x + gx_off
        base_y = port.translation.y + gy_off
        base_z = port.translation.z + z_offset + gz_off

        if use_correction:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            self._ix = np.clip(self._ix + ex, -self._max_windup, self._max_windup)
            self._iy = np.clip(self._iy + ey, -self._max_windup, self._max_windup)
            cx = self._cx_ema.update(self._i_gain * self._ix)
            cy = self._cy_ema.update(self._i_gain * self._iy)
            tx = base_x + cx
            ty = base_y + cy
        else:
            tx = base_x
            ty = base_y
        tz = base_z

        bx = pos_frac * tx + (1.0 - pos_frac) * gx
        by = pos_frac * ty + (1.0 - pos_frac) * gy
        bz = pos_frac * tz + (1.0 - pos_frac) * gz

        return Pose(
            position=Point(x=bx, y=by, z=bz),
            orientation=Quaternion(w=q_blend[0], x=q_blend[1],
                                  y=q_blend[2], z=q_blend[3]),
        )

    def _send(self, move_robot, pose):
        self.set_pose_target(move_robot=move_robot, pose=pose)

    def _approach(self, move_robot, send_feedback):
        send_feedback("Approaching...")
        n_steps = int(APPROACH_TIME / DT)
        for i in range(n_steps):
            frac = min_jerk(i / n_steps)
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(DT)
                continue
            pose = self._compute_pose(port, plug, grip, APPROACH_HEIGHT,
                                      slerp_frac=frac, pos_frac=frac,
                                      use_correction=False)
            self._send(move_robot, pose)
            self.sleep_for(DT)

        self.get_logger().info("Approach done, settling...")
        n_settle = int(APPROACH_SETTLE / DT)
        for _ in range(n_settle):
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(DT)
                continue
            pose = self._compute_pose(port, plug, grip, APPROACH_HEIGHT,
                                      slerp_frac=1.0, pos_frac=1.0,
                                      use_correction=False)
            self._send(move_robot, pose)
            self.sleep_for(DT)

    def _descend(self, move_robot, get_obs, send_feedback):
        params = CONNECTOR_PARAMS.get(self._task.plug_type, DEFAULT_PARAMS)
        max_depth = params["max_depth"]
        force_limit = params["force_limit"]
        total_z = APPROACH_HEIGHT - max_depth

        self._ix = 0.0
        self._iy = 0.0
        self._cx_ema.reset()
        self._cy_ema.reset()

        baseline = self._measure_baseline(get_obs)
        self.get_logger().info(f"Force baseline: {baseline:.1f}N")

        send_feedback("Inserting...")
        z_offset = APPROACH_HEIGHT
        t = 0.0
        consecutive_jams = 0

        while z_offset > max_depth:
            tau = np.clip(t / DESCENT_TIME, 0.0, 1.0)
            if tau < 1.0:
                speed = min_jerk_vel(tau) * total_z / DESCENT_TIME
            else:
                speed = 0.5 * total_z / DESCENT_TIME
            step = max(speed * DT, MIN_STEP)

            z_offset -= step
            z_offset = max(z_offset, max_depth)

            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(DT)
                continue

            plug_z_before = plug.translation.z

            pose = self._compute_pose(port, plug, grip, z_offset,
                                      use_correction=True)
            self._send(move_robot, pose)
            self.sleep_for(DT)

            force = self._get_force(get_obs)
            df = force - baseline

            plug_after = self._plug_tf()
            dz = (plug_z_before - plug_after.translation.z) if plug_after else step

            if df > force_limit:
                if dz < step * 0.3:
                    consecutive_jams += 1
                    self.get_logger().warn(
                        f"JAM: df={df:.1f}N dz={dz*1000:.3f}mm "
                        f"z={z_offset:.4f} jams={consecutive_jams}"
                    )
                    z_offset += step * 3
                    pose = self._compute_pose(port, plug, grip, z_offset,
                                              use_correction=True)
                    self._send(move_robot, pose)
                    self.sleep_for(0.1)
                    if consecutive_jams >= 5:
                        self.get_logger().warn("Too many jams, stopping")
                        break
                    continue
                else:
                    self.get_logger().info(
                        f"Force df={df:.1f}N but moving dz={dz*1000:.3f}mm"
                    )
                    consecutive_jams = 0
            else:
                if consecutive_jams > 0 and df < force_limit * 0.3:
                    consecutive_jams = max(0, consecutive_jams - 1)

            t += DT

        plug = self._plug_tf()
        port = self._port_tf()
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            z_gap = plug.translation.z - port.translation.z
            self.get_logger().info(
                f"Final: xy={math.sqrt(ex**2+ey**2)*1000:.2f}mm "
                f"z_gap={z_gap*1000:.2f}mm"
            )

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        self.get_logger().info(f"V3.insert_cable() task: {task}")
        self._task = task
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        self._ix = 0.0
        self._iy = 0.0
        self._cx_ema.reset()
        self._cy_ema.reset()

        for frame in [self._port_frame, self._plug_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' unavailable, aborting")
                return False

        self._approach(move_robot, send_feedback)
        self._descend(move_robot, get_observation, send_feedback)

        self.get_logger().info("Stabilizing...")
        self.sleep_for(SETTLE_TIME)
        self.get_logger().info("V3.insert_cable() done")
        return True