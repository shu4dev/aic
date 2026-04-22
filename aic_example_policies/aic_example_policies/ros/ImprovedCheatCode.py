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
    "sfp": {
        "max_depth": -0.022,
        "coarse_step": 0.0012,
        "fine_step": 0.0003,
        "fine_threshold": 0.01,
        "force_limit": 15.0,
        "force_ref": 4.0,
        "force_soft": 8.0,
    },
    "sc": {
        "max_depth": -0.020,
        "coarse_step": 0.0010,
        "fine_step": 0.0003,
        "fine_threshold": 0.008,
        "force_limit": 12.0,
        "force_ref": 3.5,
        "force_soft": 7.0,
    },
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
        self._prev_grip = None
        self._prev_t = None
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
            grip = self._grip_tf()
            if grip is not None:
                self._log_vel(grip, "baseline")
            samples.append(self._get_force(get_obs))
            self.sleep_for(0.02)
        return sum(samples) / len(samples)

    def _reset_vel_tracker(self):
        self._prev_grip = None
        self._prev_t = None

    def _log_vel(self, grip, tag):
        now = self.time_now().nanoseconds / 1e9
        if self._prev_grip is not None and self._prev_t is not None:
            dt = now - self._prev_t
            if dt > 0.001:
                dx = grip.translation.x - self._prev_grip[0]
                dy = grip.translation.y - self._prev_grip[1]
                dz = grip.translation.z - self._prev_grip[2]
                vx = dx / dt
                vy = dy / dt
                vz = dz / dt
                v = math.sqrt(vx * vx + vy * vy + vz * vz)
                self.get_logger().info(
                    f"[vel:{tag}] v={v*1000:.2f} vx={vx*1000:.2f} vy={vy*1000:.2f} vz={vz*1000:.2f} dt={dt*1000:.1f}"
                )
        self._prev_grip = (grip.translation.x, grip.translation.y, grip.translation.z)
        self._prev_t = now

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
        for t in range(150):
            frac = 0.5 * (1.0 - math.cos(math.pi * t / 150))
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(0.03)
                continue
            self._log_vel(grip, "approach")
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
        align_xy_threshold = 0.0015
        plug_vel_threshold = 0.0003
        required_stable = 3
        stable_count = 0
        aligned = False
        prev_plug_xy = None

        send_feedback("Aligning...")
        while align_iters < max_align_iters:
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(0.03)
                continue

            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            xy_err = math.sqrt(ex**2 + ey**2)

            if prev_plug_xy is not None:
                dx = plug.translation.x - prev_plug_xy[0]
                dy = plug.translation.y - prev_plug_xy[1]
                plug_vel = math.sqrt(dx**2 + dy**2)
            else:
                plug_vel = 1.0
            prev_plug_xy = (plug.translation.x, plug.translation.y)

            align_iters += 1
            if align_iters % 30 == 0:
                self.get_logger().info(
                    f"[align {align_iters}] xy={xy_err*1000:.2f}mm v={plug_vel*1000:.2f}mm"
                )

            self._log_vel(grip, "align")
            pose = self._calc_pose(port, plug, grip, z_offset)
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.03)

            if xy_err < align_xy_threshold and plug_vel < plug_vel_threshold:
                stable_count += 1
                if stable_count >= required_stable:
                    aligned = True
                    break
            else:
                stable_count = 0

        port = self._port_tf()
        plug = self._plug_tf()
        if port and plug:
            final_xy_err = math.sqrt(
                (port.translation.x - plug.translation.x) ** 2
                + (port.translation.y - plug.translation.y) ** 2
            )
        else:
            final_xy_err = 999.0

        if self._approach_only:
            self.get_logger().info(
                f"Approach-only mode: xy={final_xy_err*1000:.2f}mm, aligned={aligned}"
            )
            return z_offset

        if final_xy_err > 0.006:
            self.get_logger().warn(
                f"Alignment did not converge (xy={final_xy_err*1000:.2f}mm), "
                f"holding instead of descending"
            )
            send_feedback("Alignment failed, holding")
            for _ in range(20):
                port = self._port_tf()
                plug = self._plug_tf()
                grip = self._grip_tf()
                if all([port, plug, grip]):
                    self._log_vel(grip, "align_fail_hold")
                    pose = self._calc_pose(port, plug, grip, z_offset)
                    self.set_pose_target(move_robot=move_robot, pose=pose)
                self.sleep_for(0.04)
            return z_offset

        self.get_logger().info(
            f"Starting descent: xy={final_xy_err*1000:.2f}mm after {align_iters} iters"
        )
        send_feedback("Descending...")

        consecutive_jams = 0

        while True:
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if not all([port, plug, grip]):
                self.sleep_for(0.03)
                continue

            base_step = params["coarse_step"] if z_offset > params["fine_threshold"] else params["fine_step"]

            force_now = self._get_force(get_obs)
            df_now = max(0.0, force_now - baseline)
            F_ref = params["force_ref"]
            F_soft = params["force_soft"]

            if df_now <= F_ref:
                step = base_step
            elif df_now < F_soft:
                scale = 1.0 - (df_now - F_ref) / (F_soft - F_ref)
                step = base_step * max(0.1, scale)
            else:
                step = -base_step * 0.5

            z_offset -= step

            if z_offset < params["max_depth"]:
                self.get_logger().info("Reached max depth")
                break

            plug_z_before = plug.translation.z

            self._log_vel(grip, "descent")
            pose = self._calc_pose(port, plug, grip, z_offset)
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.04)

            force = self._get_force(get_obs)
            df = force - baseline

            plug_after = self._plug_tf()
            dz = (plug_z_before - plug_after.translation.z) if plug_after else step

            if df > params["force_limit"]:
                if dz < base_step * 0.3:
                    consecutive_jams += 1
                    self.get_logger().warn(
                        f"JAM: df={df:.1f}N dz={dz*1000:.3f}mm z={z_offset:.4f} jams={consecutive_jams}"
                    )
                    z_offset += base_step * 3
                    grip_now = self._grip_tf()
                    if grip_now is not None:
                        self._log_vel(grip_now, "jam_backoff")
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

            if z_offset % 0.005 < base_step:
                self.get_logger().info(
                    f"[desc] z={z_offset:.4f} df={df:.1f}N dz={dz*1000:.3f}mm step={step*1000:.3f}mm"
                )

        plug = self._plug_tf()
        port = self._port_tf()
        if plug and port:
            ex = port.translation.x - plug.translation.x
            ey = port.translation.y - plug.translation.y
            z_gap = plug.translation.z - port.translation.z
            self.get_logger().info(
                f"Done: xy={math.sqrt(ex**2+ey**2)*1000:.2f}mm z_gap={z_gap*1000:.2f}mm"
            )

        return z_offset

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        mode = "APPROACH-ONLY (no insertion)" if self._approach_only else "FULL (approach + insert)"
        self.get_logger().info(f"ImprovedCheatCode.insert_cable() mode={mode} task: {task}")
        self._task = task
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        self._ix = 0.0
        self._iy = 0.0
        self._reset_vel_tracker()

        for frame in [self._port_frame, self._plug_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' unavailable, aborting")
                return False

        self._approach(move_robot, send_feedback)
        final_z = self._align_and_descend(move_robot, get_observation, send_feedback)

        self.get_logger().info("Stabilizing...")
        for _ in range(20):
            port = self._port_tf()
            plug = self._plug_tf()
            grip = self._grip_tf()
            if all([port, plug, grip]):
                self._log_vel(grip, "hold")
                pose = self._calc_pose(port, plug, grip, final_z)
                self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.04)

        self.get_logger().info("ImprovedCheatCode.insert_cable() exiting...")
        return True