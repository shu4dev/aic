import math
import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

QuaternionTuple = tuple[float, float, float, float]

CONNECTOR_PARAMS = {
    "sfp": {
        "max_insertion_depth": -0.020,
        "coarse_step": 0.0015,
        "fine_step": 0.0003,
        "fine_threshold": 0.01,
        "force_limit": 15.0,
    },
    "sc": {
        "max_insertion_depth": -0.018,
        "coarse_step": 0.0012,
        "fine_step": 0.0003,
        "fine_threshold": 0.008,
        "force_limit": 12.0,
    },
}

DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]


class ImprovedCheatCode(Policy):
    def __init__(self, parent_node):
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._max_integrator_windup = 0.05
        self._task = None
        super().__init__(parent_node)

    def _wait_for_tf(
        self, target_frame: str, source_frame: str, timeout_sec: float = 15.0
    ) -> bool:
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(),
                )
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for transform '{source_frame}' -> '{target_frame}'..."
                    )
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(
            f"Transform '{source_frame}' not available after {timeout_sec}s"
        )
        return False

    def _lookup_port_transform(self, port_frame: str) -> Transform | None:
        try:
            tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link", port_frame, Time(),
            )
            return tf_stamped.transform
        except TransformException as ex:
            self.get_logger().warn(f"Port TF lookup failed: {ex}")
            return None

    def _get_force_magnitude(self, get_observation: GetObservationCallback) -> float:
        obs = get_observation()
        if obs is None or obs.wrist_wrench is None:
            return 0.0
        f = obs.wrist_wrench.wrench.force
        return math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2)

    def calc_gripper_pose(
        self,
        port_transform: Transform,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
        xy_spiral_offset: tuple[float, float] = (0.0, 0.0),
    ) -> Pose:
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )
        plug_tf = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            f"{self._task.cable_name}/{self._task.plug_name}_link",
            Time(),
        )
        q_plug = (
            plug_tf.transform.rotation.w,
            plug_tf.transform.rotation.x,
            plug_tf.transform.rotation.y,
            plug_tf.transform.rotation.z,
        )
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)

        gripper_tf = self._parent_node._tf_buffer.lookup_transform(
            "base_link", "gripper/tcp", Time(),
        )
        q_gripper = (
            gripper_tf.transform.rotation.w,
            gripper_tf.transform.rotation.x,
            gripper_tf.transform.rotation.y,
            gripper_tf.transform.rotation.z,
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)
        q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_target, slerp_fraction)

        gripper_xyz = (
            gripper_tf.transform.translation.x,
            gripper_tf.transform.translation.y,
            gripper_tf.transform.translation.z,
        )
        port_xy = (
            port_transform.translation.x,
            port_transform.translation.y,
        )
        plug_xyz = (
            plug_tf.transform.translation.x,
            plug_tf.transform.translation.y,
            plug_tf.transform.translation.z,
        )
        plug_tip_gripper_offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        tip_x_error = port_xy[0] - plug_xyz[0]
        tip_y_error = port_xy[1] - plug_xyz[1]

        if reset_xy_integrator:
            self._tip_x_error_integrator = 0.0
            self._tip_y_error_integrator = 0.0
        else:
            self._tip_x_error_integrator = np.clip(
                self._tip_x_error_integrator + tip_x_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )
            self._tip_y_error_integrator = np.clip(
                self._tip_y_error_integrator + tip_y_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )

        i_gain = 0.2

        target_x = port_xy[0] + i_gain * self._tip_x_error_integrator + xy_spiral_offset[0]
        target_y = port_xy[1] + i_gain * self._tip_y_error_integrator + xy_spiral_offset[1]
        target_z = port_transform.translation.z + z_offset - plug_tip_gripper_offset[2]

        blend_xyz = (
            position_fraction * target_x + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_y + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_z + (1.0 - position_fraction) * gripper_xyz[2],
        )

        return Pose(
            position=Point(x=blend_xyz[0], y=blend_xyz[1], z=blend_xyz[2]),
            orientation=Quaternion(
                w=q_gripper_slerp[0],
                x=q_gripper_slerp[1],
                y=q_gripper_slerp[2],
                z=q_gripper_slerp[3],
            ),
        )

    def _do_approach(
        self,
        port_frame: str,
        port_transform: Transform,
        move_robot: MoveRobotCallback,
        z_offset: float,
    ):
        n_steps = 40
        for t in range(n_steps):
            frac = 0.5 * (1.0 - math.cos(math.pi * t / n_steps))
            fresh_tf = self._lookup_port_transform(port_frame)
            if fresh_tf is not None:
                port_transform = fresh_tf
            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(
                        port_transform,
                        slerp_fraction=frac,
                        position_fraction=frac,
                        z_offset=z_offset,
                        reset_xy_integrator=True,
                    ),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF failed during approach: {ex}")
            self.sleep_for(0.03)
        return port_transform

    def _do_spiral_search(
        self,
        port_frame: str,
        port_transform: Transform,
        move_robot: MoveRobotCallback,
        get_observation: GetObservationCallback,
        z_offset: float,
        params: dict,
    ) -> tuple[bool, Transform, float]:
        self.get_logger().info("Starting spiral search recovery...")
        max_radius = 0.003
        n_spiral_steps = 60
        growth_rate = max_radius / n_spiral_steps

        for step in range(n_spiral_steps):
            radius = growth_rate * step
            angle = step * 0.5
            spiral_x = radius * math.cos(angle)
            spiral_y = radius * math.sin(angle)

            fresh_tf = self._lookup_port_transform(port_frame)
            if fresh_tf is not None:
                port_transform = fresh_tf

            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(
                        port_transform,
                        z_offset=z_offset,
                        xy_spiral_offset=(spiral_x, spiral_y),
                    ),
                )
            except TransformException:
                pass
            self.sleep_for(0.03)

            force = self._get_force_magnitude(get_observation)
            if force < params["force_limit"] * 0.5:
                self.get_logger().info(f"Spiral found low-force path at step {step}")
                return True, port_transform, z_offset

        self.get_logger().warn("Spiral search did not find clear path")
        return False, port_transform, z_offset

    def _do_descent(
        self,
        port_frame: str,
        port_transform: Transform,
        move_robot: MoveRobotCallback,
        get_observation: GetObservationCallback,
        z_offset: float,
        params: dict,
    ) -> tuple[Transform, float]:
        consecutive_jams = 0
        max_retries = 3

        while z_offset > params["max_insertion_depth"]:
            if z_offset > params["fine_threshold"]:
                step = params["coarse_step"]
            else:
                step = params["fine_step"]

            z_offset -= step

            fresh_tf = self._lookup_port_transform(port_frame)
            if fresh_tf is not None:
                port_transform = fresh_tf

            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(port_transform, z_offset=z_offset),
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF failed during descent: {ex}")
            self.sleep_for(0.03)

            force = self._get_force_magnitude(get_observation)

            if force > params["force_limit"]:
                self.get_logger().warn(
                    f"Force {force:.1f}N exceeds limit {params['force_limit']}N at z_offset={z_offset:.4f}"
                )
                z_offset += step * 3
                try:
                    self.set_pose_target(
                        move_robot=move_robot,
                        pose=self.calc_gripper_pose(port_transform, z_offset=z_offset),
                    )
                except TransformException:
                    pass
                self.sleep_for(0.1)

                self._tip_x_error_integrator = 0.0
                self._tip_y_error_integrator = 0.0

                for settle_step in range(15):
                    fresh_tf = self._lookup_port_transform(port_frame)
                    if fresh_tf is not None:
                        port_transform = fresh_tf
                    try:
                        self.set_pose_target(
                            move_robot=move_robot,
                            pose=self.calc_gripper_pose(port_transform, z_offset=z_offset),
                        )
                    except TransformException:
                        pass
                    self.sleep_for(0.03)

                consecutive_jams += 1
                if consecutive_jams >= max_retries:
                    self.get_logger().info("Multiple jams detected, trying spiral search")
                    success, port_transform, z_offset = self._do_spiral_search(
                        port_frame, port_transform, move_robot, get_observation,
                        z_offset, params,
                    )
                    consecutive_jams = 0
                    if not success:
                        z_offset += 0.005
            else:
                if consecutive_jams > 0 and force < params["force_limit"] * 0.3:
                    consecutive_jams = max(0, consecutive_jams - 1)

        return port_transform, z_offset

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"ImprovedCheatCode.insert_cable() task: {task}")
        self._task = task
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0

        params = CONNECTOR_PARAMS.get(task.plug_type, DEFAULT_PARAMS)
        self.get_logger().info(f"Using params for plug_type='{task.plug_type}': {params}")

        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"

        for frame in [port_frame, cable_tip_frame]:
            if not self._wait_for_tf("base_link", frame):
                send_feedback(f"TF '{frame}' not available, aborting")
                return False

        port_transform = self._lookup_port_transform(port_frame)
        if port_transform is None:
            return False

        z_offset = 0.05

        send_feedback("Approaching port...")
        port_transform = self._do_approach(
            port_frame, port_transform, move_robot, z_offset,
        )

        for settle in range(10):
            fresh_tf = self._lookup_port_transform(port_frame)
            if fresh_tf is not None:
                port_transform = fresh_tf
            try:
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=self.calc_gripper_pose(
                        port_transform,
                        z_offset=z_offset,
                        reset_xy_integrator=False,
                    ),
                )
            except TransformException:
                pass
            self.sleep_for(0.03)

        send_feedback("Descending with force monitoring...")
        port_transform, z_offset = self._do_descent(
            port_frame, port_transform, move_robot, get_observation,
            z_offset, params,
        )

        self.get_logger().info("Waiting for connector to stabilize...")
        self.sleep_for(1.5)

        self.get_logger().info("ImprovedCheatCode.insert_cable() exiting...")
        return True
