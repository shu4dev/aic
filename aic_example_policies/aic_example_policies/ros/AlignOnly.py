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

ALIGN_HEIGHT = 0.05
SETTLE_TIME = 0.8
SETTLE_STEPS = 20
XY_TOLERANCE = 0.001
ORIENTATION_TOLERANCE = 0.02


class AlignOnly(Policy):
    def __init__(self, parent_node):
        self._task = None
        super().__init__(parent_node)

    def _wait_for_tf(self, target_frame: str, source_frame: str, timeout_sec: float = 15.0) -> bool:
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(target_frame, source_frame, Time())
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(f"Waiting for TF '{source_frame}' -> '{target_frame}'...")
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(f"TF '{source_frame}' not available after {timeout_sec}s")
        return False

    def _lookup(self, target: str, source: str) -> Transform | None:
        try:
            return self._parent_node._tf_buffer.lookup_transform(target, source, Time()).transform
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return None

    def _get_port_tf(self) -> Transform | None:
        return self._lookup("base_link", self._port_frame)

    def _get_plug_tf(self) -> Transform | None:
        return self._lookup("base_link", self._plug_frame)

    def _get_gripper_tf(self) -> Transform | None:
        return self._lookup("base_link", "gripper/tcp")

    def _quat_to_tuple(self, rot) -> QuaternionTuple:
        return (rot.w, rot.x, rot.y, rot.z)

    def _plug_gripper_offset(self, gripper_tf: Transform, plug_tf: Transform) -> tuple[float, float, float]:
        return (
            gripper_tf.translation.x - plug_tf.translation.x,
            gripper_tf.translation.y - plug_tf.translation.y,
            gripper_tf.translation.z - plug_tf.translation.z,
        )

    def _orientation_error(self, plug_tf: Transform, port_tf: Transform) -> float:
        q_port = self._quat_to_tuple(port_tf.rotation)
        q_plug = self._quat_to_tuple(plug_tf.rotation)
        dot = abs(q_port[0]*q_plug[0] + q_port[1]*q_plug[1] + q_port[2]*q_plug[2] + q_port[3]*q_plug[3])
        dot = min(dot, 1.0)
        return 2.0 * math.acos(dot)

    def _xy_error(self, plug_tf: Transform, port_tf: Transform) -> tuple[float, float]:
        return (
            port_tf.translation.x - plug_tf.translation.x,
            port_tf.translation.y - plug_tf.translation.y,
        )

    def _settle(self, move_robot, pose, steps=SETTLE_STEPS, dt=0.03):
        for _ in range(steps):
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(dt)

    def _phase_move_above(self, move_robot: MoveRobotCallback, send_feedback: SendFeedbackCallback):
        send_feedback("Phase 1: Moving above port...")
        n_steps = 60

        for t in range(n_steps):
            frac = 0.5 * (1.0 - math.cos(math.pi * t / n_steps))

            port_tf = self._get_port_tf()
            plug_tf = self._get_plug_tf()
            gripper_tf = self._get_gripper_tf()
            if not all([port_tf, plug_tf, gripper_tf]):
                self.sleep_for(0.03)
                continue

            offset = self._plug_gripper_offset(gripper_tf, plug_tf)

            target_x = port_tf.translation.x + offset[0]
            target_y = port_tf.translation.y + offset[1]
            target_z = port_tf.translation.z + ALIGN_HEIGHT + offset[2]

            gx, gy, gz = gripper_tf.translation.x, gripper_tf.translation.y, gripper_tf.translation.z
            pose = Pose(
                position=Point(
                    x=gx + frac * (target_x - gx),
                    y=gy + frac * (target_y - gy),
                    z=gz + frac * (target_z - gz),
                ),
                orientation=Quaternion(
                    w=gripper_tf.rotation.w,
                    x=gripper_tf.rotation.x,
                    y=gripper_tf.rotation.y,
                    z=gripper_tf.rotation.z,
                ),
            )
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.04)

        self.get_logger().info("Phase 1 done. Settling...")
        self.sleep_for(SETTLE_TIME)

    def _phase_orient(self, move_robot: MoveRobotCallback, send_feedback: SendFeedbackCallback):
        send_feedback("Phase 2: Orienting plug to match port...")
        n_steps = 80

        for t in range(n_steps):
            frac = 0.5 * (1.0 - math.cos(math.pi * t / n_steps))

            port_tf = self._get_port_tf()
            plug_tf = self._get_plug_tf()
            gripper_tf = self._get_gripper_tf()
            if not all([port_tf, plug_tf, gripper_tf]):
                self.sleep_for(0.03)
                continue

            q_port = self._quat_to_tuple(port_tf.rotation)
            q_plug = self._quat_to_tuple(plug_tf.rotation)
            q_gripper = self._quat_to_tuple(gripper_tf.rotation)

            q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
            q_diff = quaternion_multiply(q_port, q_plug_inv)
            q_gripper_target = quaternion_multiply(q_diff, q_gripper)
            q_blended = quaternion_slerp(q_gripper, q_gripper_target, frac)

            offset = self._plug_gripper_offset(gripper_tf, plug_tf)
            pose = Pose(
                position=Point(
                    x=port_tf.translation.x + offset[0],
                    y=port_tf.translation.y + offset[1],
                    z=port_tf.translation.z + ALIGN_HEIGHT + offset[2],
                ),
                orientation=Quaternion(w=q_blended[0], x=q_blended[1], y=q_blended[2], z=q_blended[3]),
            )
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.04)

        self.get_logger().info("Phase 2 done. Settling...")
        self.sleep_for(SETTLE_TIME)

    def _phase_fine_align(self, move_robot: MoveRobotCallback, send_feedback: SendFeedbackCallback) -> bool:
        send_feedback("Phase 3: Fine lateral alignment...")
        max_iterations = 150
        p_gain = 0.6

        for i in range(max_iterations):
            port_tf = self._get_port_tf()
            plug_tf = self._get_plug_tf()
            gripper_tf = self._get_gripper_tf()
            if not all([port_tf, plug_tf, gripper_tf]):
                self.sleep_for(0.03)
                continue

            ex, ey = self._xy_error(plug_tf, port_tf)
            orient_err = self._orientation_error(plug_tf, port_tf)
            xy_dist = math.sqrt(ex**2 + ey**2)

            if i % 20 == 0:
                self.get_logger().info(
                    f"[align iter {i}] xy_err={xy_dist*1000:.2f}mm  orient_err={math.degrees(orient_err):.2f}deg"
                )

            if xy_dist < XY_TOLERANCE and orient_err < ORIENTATION_TOLERANCE:
                self.get_logger().info(
                    f"ALIGNED at iter {i}: xy_err={xy_dist*1000:.3f}mm  orient_err={math.degrees(orient_err):.3f}deg"
                )
                return True

            offset = self._plug_gripper_offset(gripper_tf, plug_tf)

            q_port = self._quat_to_tuple(port_tf.rotation)
            q_plug = self._quat_to_tuple(plug_tf.rotation)
            q_gripper = self._quat_to_tuple(gripper_tf.rotation)
            q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
            q_diff = quaternion_multiply(q_port, q_plug_inv)
            q_target = quaternion_multiply(q_diff, q_gripper)
            q_corrected = quaternion_slerp(q_gripper, q_target, 0.15)

            pose = Pose(
                position=Point(
                    x=port_tf.translation.x + offset[0] + p_gain * ex,
                    y=port_tf.translation.y + offset[1] + p_gain * ey,
                    z=port_tf.translation.z + ALIGN_HEIGHT + offset[2],
                ),
                orientation=Quaternion(w=q_corrected[0], x=q_corrected[1], y=q_corrected[2], z=q_corrected[3]),
            )
            self.set_pose_target(move_robot=move_robot, pose=pose)
            self.sleep_for(0.03)

        port_tf = self._get_port_tf()
        plug_tf = self._get_plug_tf()
        if port_tf and plug_tf:
            ex, ey = self._xy_error(plug_tf, port_tf)
            orient_err = self._orientation_error(plug_tf, port_tf)
            self.get_logger().warn(
                f"Fine align exhausted: xy_err={math.sqrt(ex**2+ey**2)*1000:.2f}mm  orient_err={math.degrees(orient_err):.2f}deg"
            )
        return False

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"AlignOnly.insert_cable() task: {task}")
        self._task = task
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"

        for frame in [self._port_frame, self._plug_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' unavailable, aborting")
                return False

        self._phase_move_above(move_robot, send_feedback)
        self._phase_orient(move_robot, send_feedback)
        aligned = self._phase_fine_align(move_robot, send_feedback)

        port_tf = self._get_port_tf()
        plug_tf = self._get_plug_tf()
        gripper_tf = self._get_gripper_tf()
        if all([port_tf, plug_tf, gripper_tf]):
            ex, ey = self._xy_error(plug_tf, port_tf)
            orient_err = self._orientation_error(plug_tf, port_tf)
            z_gap = plug_tf.translation.z - port_tf.translation.z
            self.get_logger().info("=" * 60)
            self.get_logger().info("ALIGNMENT REPORT")
            self.get_logger().info(f"  XY error:          {math.sqrt(ex**2+ey**2)*1000:.3f} mm")
            self.get_logger().info(f"  X error:           {ex*1000:.3f} mm")
            self.get_logger().info(f"  Y error:           {ey*1000:.3f} mm")
            self.get_logger().info(f"  Z gap above port:  {z_gap*1000:.3f} mm")
            self.get_logger().info(f"  Orientation error: {math.degrees(orient_err):.3f} deg")
            self.get_logger().info(f"  Aligned:           {aligned}")
            self.get_logger().info("=" * 60)

        send_feedback("Alignment complete. Holding position.")
        self.sleep_for(3.0)

        self.get_logger().info("AlignOnly.insert_cable() exiting...")
        return True
