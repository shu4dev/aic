import time
import numpy as np
import torch
import cv2
from PIL import Image
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from aic_model.policy import (
    GetObservationCallback, MoveRobotCallback, Policy, SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from geometry_msgs.msg import Wrench

from transformers import AutoModelForVision2Seq, AutoProcessor


class RunOpenVLA(Policy):
    def __init__(self, parent_node: Node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        model_id = "openvla/openvla-7b"
        self.get_logger().info(f"Loading OpenVLA from {model_id}...")

        self.processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
        self.vla = AutoModelForVision2Seq.from_pretrained(
            model_id,
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        ).to(self.device)

        self.get_logger().info(f"OpenVLA loaded on {self.device}")
        self.task_instruction = "insert the cable into the port"

        # Set this to your fine-tuned dataset key for un-normalization,
        # or None to get raw normalized actions
        self.unnorm_key = "berkeley_autolab_ur5"  # e.g., "bridge_orig" for BridgeV2

    def _ros_img_to_pil(self, raw_img) -> Image.Image:
        """Convert ROS Image to PIL Image."""
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        return Image.fromarray(img_np)

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info(f"RunOpenVLA.insert_cable() enter. Task: {task}")

        prompt = f"In: What action should the robot take to {self.task_instruction}?\nOut:"

        start_time = time.time()

        while time.time() - start_time < 30.0:
            loop_start = time.time()

            obs_msg = get_observation()
            if obs_msg is None:
                continue

            # OpenVLA takes a single image
            pil_image = self._ros_img_to_pil(obs_msg.center_image)

            # Run inference
            inputs = self.processor(prompt, pil_image).to(self.device, dtype=torch.bfloat16)
            with torch.inference_mode():
                action = self.vla.predict_action(
                    **inputs,
                    unnorm_key=self.unnorm_key,
                    do_sample=False,
                )
            # action is a numpy array of 7 values

            self.get_logger().info(f"OpenVLA action: {action}")

            # Apply scaling factor (OpenVLA outputs are small deltas)
            scale = 1.0  # Tune this for your setup
            twist = Twist(
                linear=Vector3(
                    x=float(action[0] * scale),
                    y=float(action[1] * scale),
                    z=float(action[2] * scale),
                ),
                angular=Vector3(
                    x=float(action[3] * scale),
                    y=float(action[4] * scale),
                    z=float(action[5] * scale),
                ),
            )
            motion_update = self._make_velocity_command(twist)
            move_robot(motion_update=motion_update)
            send_feedback("openvla inference in progress...")

            # OpenVLA is slower (~1-3 Hz), adjust sleep accordingly
            elapsed = time.time() - loop_start
            time.sleep(max(0, 0.5 - elapsed))

        self.get_logger().info("RunOpenVLA.insert_cable() exiting...")
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
