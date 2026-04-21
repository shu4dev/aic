import math
import os
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

CONNECTOR_PARAMS = {
    "sfp": {
        "max_depth": -0.022,
        "approach_z": 0.05,
        "align_z": 0.015,
        "force_ref": 6.0,
        "force_max": 16.0,
        "v_max": 0.010,
        "T_approach": 2.5,
        "T_align": 1.2,
    },
    "sc": {
        "max_depth": -0.020,
        "approach_z": 0.05,
        "align_z": 0.012,
        "force_ref": 5.0,
        "force_max": 14.0,
        "v_max": 0.008,
        "T_approach": 2.5,
        "T_align": 1.2,
    },
}
DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]

CTRL_DT = 0.01
SETTLE_TIME = 0.8


class ImprovedCheatCode(Policy):
    def __init__(self, parent_node):
        self._task = None
        self._approach_only = os.environ.get("CHEATCODE_APPROACH_ONLY", "0") == "1"
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

    def _get_force(self, get_obs):
        obs = get_obs()
        if obs is None or obs.wrist_wrench is None:
            return 0.0
        f = obs.wrist_wrench.wrench.force
        return math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2)

    def _measure_baseline(self, get_obs, n=10):
        s = 0.0
        for _ in range(n):
            s += self._get_force(get_obs)
            self.sleep_for(0.02)
        return s / n

    @staticmethod
    def _skew(w):
        return np.array([
            [0.0, -w[2], w[1]],
            [w[2], 0.0, -w[0]],
            [-w[1], w[0], 0.0],
        ])

    @staticmethod
    def _quat_to_mat(q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])

    @staticmethod
    def _mat_to_quat(R):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            return (0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S)
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            return ((R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S)
        if R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            return ((R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S)
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        return ((R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S)

    def _tf_to_mat(self, tf):
        T = np.eye(4)
        T[:3, :3] = self._quat_to_mat([tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])
        T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
        return T

    def _mat_to_pose(self, T):
        q = self._mat_to_quat(T[:3, :3])
        return Pose(
            position=Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
            orientation=Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3])),
        )

    def _so3_log(self, R):
        cos_t = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
        theta = math.acos(cos_t)
        if theta < 1e-9:
            return np.zeros(3)
        if math.pi - theta < 1e-6:
            d = np.diag(R)
            idx = int(np.argmax(d))
            a = np.zeros(3)
            a[idx] = math.sqrt(max(0.0, (d[idx] + 1.0) * 0.5))
            for i in range(3):
                if i != idx and a[idx] > 1e-9:
                    a[i] = R[idx, i] / (2 * a[idx])
            return a * theta
        sin_t = math.sin(theta)
        omega = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ]) / (2 * sin_t)
        return omega * theta

    def _se3_log(self, T):
        omega = self._so3_log(T[:3, :3])
        theta = float(np.linalg.norm(omega))
        p = T[:3, 3]
        if theta < 1e-9:
            return np.concatenate([omega, p])
        K = self._skew(omega / theta)
        J = (np.eye(3)
             + (1 - math.cos(theta)) / theta * K
             + (theta - math.sin(theta)) / theta * (K @ K))
        v = np.linalg.solve(J, p)
        return np.concatenate([omega, v])

    def _se3_exp(self, xi):
        omega = xi[:3]
        v = xi[3:]
        theta = float(np.linalg.norm(omega))
        T = np.eye(4)
        if theta < 1e-9:
            T[:3, 3] = v
            return T
        K = self._skew(omega / theta)
        R = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)
        J = (np.eye(3)
             + (1 - math.cos(theta)) / theta * K
             + (theta - math.sin(theta)) / theta * (K @ K))
        T[:3, :3] = R
        T[:3, 3] = J @ v
        return T

    def _sclerp(self, T_start, T_end, s):
        T_rel = np.linalg.inv(T_start) @ T_end
        xi = self._se3_log(T_rel)
        return T_start @ self._se3_exp(s * xi)

    @staticmethod
    def _quintic_s(tau):
        tau = max(0.0, min(1.0, tau))
        return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5

    def _grip_target_pose(self, port_tf, plug_tf, grip_tf, z_offset):
        T_port = self._tf_to_mat(port_tf)
        T_plug = self._tf_to_mat(plug_tf)
        T_grip = self._tf_to_mat(grip_tf)
        T_port_goal = T_port.copy()
        T_port_goal[2, 3] += z_offset
        T_grip_to_plug = np.linalg.inv(T_grip) @ T_plug
        return T_port_goal @ np.linalg.inv(T_grip_to_plug)

    def _move_segment(self, move_robot, send_feedback, z_offset, T, label):
        send_feedback(f"{label}...")
        grip = self._grip_tf()
        port = self._port_tf()
        plug = self._plug_tf()
        if not all([grip, port, plug]):
            self.get_logger().warn(f"{label}: missing TF, skipping")
            return False

        T_start = self._tf_to_mat(grip)
        T_goal = self._grip_target_pose(port, plug, grip, z_offset)

        n = max(1, int(T / CTRL_DT))
        for i in range(n + 1):
            s = self._quintic_s(i / n)
            T_t = self._sclerp(T_start, T_goal, s)
            self.set_pose_target(move_robot=move_robot, pose=self._mat_to_pose(T_t))
            self.sleep_for(CTRL_DT)
        return True

    def _admittance_descend(self, move_robot, get_obs, send_feedback, params):
        send_feedback("Descending...")
        F_ref = params["force_ref"]
        F_max = params["force_max"]
        v_max = params["v_max"]
        z_min = params["max_depth"]

        baseline = self._measure_baseline(get_obs)
        self.get_logger().info(f"Force baseline: {baseline:.2f}N")

        z_offset = params["align_z"]
        last_log = z_offset
        overforce_streak = 0

        while z_offset > z_min:
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(CTRL_DT)
                continue

            f = max(0.0, self._get_force(get_obs) - baseline)
            v_z = v_max * np.clip((f - F_ref) / F_ref, -1.0, 1.0)

            if f > F_max:
                v_z = v_max
                overforce_streak += 1
                if overforce_streak > 30:
                    self.get_logger().warn(f"Sustained overforce ({f:.1f}N), aborting")
                    break
            else:
                overforce_streak = 0

            z_offset = max(z_min, z_offset + v_z * CTRL_DT)
            T_target = self._grip_target_pose(port, plug, grip, z_offset)
            self.set_pose_target(move_robot=move_robot, pose=self._mat_to_pose(T_target))
            self.sleep_for(CTRL_DT)

            if abs(z_offset - last_log) > 0.005:
                self.get_logger().info(f"[desc] z={z_offset*1000:.1f}mm f={f:.1f}N")
                last_log = z_offset

        plug = self._plug_tf()
        port = self._port_tf()
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            z_gap = plug.translation.z - port.translation.z
            self.get_logger().info(
                f"Descent done: xy={math.sqrt(ex**2+ey**2)*1000:.2f}mm z_gap={z_gap*1000:.2f}mm"
            )

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        mode = "APPROACH-ONLY" if self._approach_only else "FULL"
        self.get_logger().info(f"ImprovedCheatCode mode={mode} task: {task}")
        self._task = task
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"

        for frame in [self._port_frame, self._plug_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' unavailable, aborting")
                return False

        params = CONNECTOR_PARAMS.get(self._task.plug_type, DEFAULT_PARAMS)

        if not self._move_segment(move_robot, send_feedback,
                                  z_offset=params["approach_z"],
                                  T=params["T_approach"],
                                  label="Approaching"):
            return False
        self.sleep_for(SETTLE_TIME)

        if not self._move_segment(move_robot, send_feedback,
                                  z_offset=params["align_z"],
                                  T=params["T_align"],
                                  label="Aligning"):
            return False
        self.sleep_for(SETTLE_TIME)

        if self._approach_only:
            self.get_logger().info("Approach-only mode: skipping descent")
            return True

        self._admittance_descend(move_robot, get_observation, send_feedback, params)

        self.sleep_for(1.0)
        self.get_logger().info("ImprovedCheatCode exiting")
        return True
