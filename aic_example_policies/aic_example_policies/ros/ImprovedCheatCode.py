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
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

QuaternionTuple = tuple[float, float, float, float]

CONNECTOR_PARAMS = {
    "sfp": {"max_depth": -0.022, "coarse_step": 0.0012, "fine_step": 0.0003,
            "fine_threshold": 0.01, "force_limit": 15.0},
    "sc":  {"max_depth": -0.020, "coarse_step": 0.0010, "fine_step": 0.0003,
            "fine_threshold": 0.008, "force_limit": 12.0},
}
DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]


class ImprovedCheatCode(Policy):
    def __init__(self, parent_node):
        self._task = None
        self._ix = 0.0
        self._iy = 0.0
        self._max_windup = 0.05
        self._i_gain = 0.15
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

    def _calc_pose(self, port, plug, grip, z_offset,
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

        return Pose(
            position=Point(x=bx, y=by, z=bz),
            orientation=Quaternion(w=q_blend[0], x=q_blend[1], y=q_blend[2], z=q_blend[3]),
        )

    def _approach(self, move_robot, send_feedback):
        send_feedback("Approaching port...")
        for t in range(100):
            frac = 0.5 * (1.0 - math.cos(math.pi * t / 100))
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(0.03)
                continue
            pose = self._calc_pose(port, plug, grip, 0.05,
                                   slerp_frac=frac, pos_frac=frac, reset_integrator=True)
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.04)
        self.get_logger().info("Approach done, settling...")
        self.sleep_for(0.8)

    def _align_and_descend(self, move_robot, get_obs, send_feedback):
        params = CONNECTOR_PARAMS.get(self._task.plug_type, DEFAULT_PARAMS)
        self._ix = 0.0
        self._iy = 0.0

        baseline = self._measure_baseline(get_obs)
        self.get_logger().info(f"Force baseline: {baseline:.1f}N")

        z_offset = 0.05
        align_iters = 0
        max_align_iters = 200
        descending = False
        consecutive_jams = 0

        send_feedback("Aligning...")

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

            if not descending:
                align_iters += 1
                if align_iters % 30 == 0:
                    self.get_logger().info(
                        f"[align {align_iters}] xy={xy_err*1000:.2f}mm ix={self._ix*1000:.1f} iy={self._iy*1000:.1f}"
                    )
                pose = self._calc_pose(port, plug, grip, z_offset)
                self.set_pose_target(move_robot=move_robot, pose=pose)
                self.sleep_for(0.03)

                if xy_err < 0.002 or align_iters >= max_align_iters:
                    if self._approach_only:
                        self.get_logger().info(
                            f"Approach-only mode: aligned xy={xy_err*1000:.2f}mm after {align_iters} iters, skipping descent"
                        )
                        break
                    self.get_logger().info(f"Starting descent: xy={xy_err*1000:.2f}mm after {align_iters} iters")
                    descending = True
                    send_feedback("Descending...")
                continue

            step = params["coarse_step"] if z_offset > params["fine_threshold"] else params["fine_step"]
            z_offset -= step

            if z_offset < params["max_depth"]:
                self.get_logger().info("Reached max depth")
                break

            plug_z_before = plug.translation.z

            pose = self._calc_pose(port, plug, grip, z_offset)
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.04)

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
                    pose = self._calc_pose(port, plug, grip, z_offset)
                    self.set_pose_target(move_robot=move_robot, pose=pose)
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
                    f"[desc] z={z_offset:.4f} df={df:.1f}N dz={dz*1000:.3f}mm xy={xy_err*1000:.1f}mm"
                )

        plug = self._plug_tf()
        port = self._port_tf()
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            z_gap = plug.translation.z - port.translation.z
            self.get_logger().info(f"Done: xy={math.sqrt(ex**2+ey**2)*1000:.2f}mm z_gap={z_gap*1000:.2f}mm")

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

        self._approach(move_robot, send_feedback)
        self._align_and_descend(move_robot, get_observation, send_feedback)

        self.get_logger().info("Stabilizing...")
        self.sleep_for(1.5)

        self.get_logger().info("ImprovedCheatCode.insert_cable() exiting...")
        return True