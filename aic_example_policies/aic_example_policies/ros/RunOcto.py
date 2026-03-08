import time
import numpy as np
import jax
import cv2
from typing import Callable
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from aic_model.policy import (
    GetObservationCallback, MoveRobotCallback, Policy, SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from geometry_msgs.msg import Wrench

from octo.model.octo_model import OctoModel


class RunOcto(Policy):
    def __init__(self, parent_node: Node):
        super().__init__(parent_node)

        # Load pretrained Octo (octo-small is 27M params, octo-base is 93M)
        self.model = OctoModel.load_pretrained("hf://rail-berkeley/octo-base-1.5")
        self.get_logger().info(f"Octo model loaded. Spec:\n{self.model.get_pretty_spec()}")

        # Octo uses a history window of 2
        self.obs_history = []
        self.task_description = "insert the cable into the port"
        self.image_size = 256

    def _ros_img_to_numpy(self, raw_img) -> np.ndarray:
        """Convert ROS Image to (H, W, 3) uint8 numpy array, resized to 256x256."""
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        img_np = cv2.resize(img_np, (self.image_size, self.image_size),
                            interpolation=cv2.INTER_AREA)
        return img_np

    def prepare_octo_observation(self, obs_msg: Observation) -> dict:
        """Build Octo observation dict from a ROS Observation message.

        Octo expects:
          observation["image_primary"]: (batch, horizon, H, W, 3) uint8
          observation["timestep_pad_mask"]: (batch, horizon) bool
        """
        # Use center camera as primary (you can also add wrist cam as image_wrist)
        center_img = self._ros_img_to_numpy(obs_msg.center_image)

        self.obs_history.append(center_img)
        # Keep only last 2 (Octo's window size)
        if len(self.obs_history) > 2:
            self.obs_history = self.obs_history[-2:]

        # Pad if we don't have 2 observations yet
        window_size = 2
        images = list(self.obs_history)
        pad_mask = [True] * len(images)
        while len(images) < window_size:
            images.insert(0, images[0])
            pad_mask.insert(0, False)

        observation = {
            "image_primary": np.stack(images)[np.newaxis],  # (1, 2, 256, 256, 3)
            "timestep_pad_mask": np.array([pad_mask]),       # (1, 2)
        }
        return observation

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info(f"RunOcto.insert_cable() enter. Task: {task}")
        self.obs_history = []

        # Build language task specification
        task_spec = self.model.create_tasks(texts=[self.task_description])

        start_time = time.time()

        while time.time() - start_time < 30.0:
            loop_start = time.time()

            obs_msg = get_observation()
            if obs_msg is None:
                continue

            observation = self.prepare_octo_observation(obs_msg)

            # Sample actions from Octo (returns (batch, pred_horizon, action_dim))
            actions = self.model.sample_actions(
                observation,
                task_spec,
                rng=jax.random.PRNGKey(int(time.time() * 1000) % 2**31),
            )
            # Take first action from the chunk: (7,)
            action = np.array(actions[0, 0])

            self.get_logger().info(f"Octo action: {action}")

            # Map to velocity command (Octo outputs 7-DoF EEF deltas)
            twist = Twist(
                linear=Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2])),
                angular=Vector3(x=float(action[3]), y=float(action[4]), z=float(action[5])),
            )
            motion_update = self._make_velocity_command(twist)
            move_robot(motion_update=motion_update)
            send_feedback("octo inference in progress...")

            elapsed = time.time() - loop_start
            time.sleep(max(0, 0.25 - elapsed))

        self.get_logger().info("RunOcto.insert_cable() exiting...")
        return True

    def _make_velocity_command(self, twist: Twist, frame_id: str = "base_link"):
        msg = MotionUpdate()
        msg.velocity = twist
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_stiffness = np.diag([100., 100., 100., 50., 50., 50.]).flatten()
        msg.target_damping = np.diag([40., 40., 40., 15., 15., 15.]).flatten()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0., y=0., z=0.), torque=Vector3(x=0., y=0., z=0.))
        msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        return msg