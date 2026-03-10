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

        # Set this to your fine-tuned dataset key for un-normalization,
        # or None to get raw normalized actions
        self.unnorm_key = "berkeley_autolab_ur5"  # e.g., "bridge_orig" for BridgeV2

    def _ros_img_to_pil(self, raw_img) -> Image.Image:
        """Convert ROS Image to PIL Image."""
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        return Image.fromarray(img_np)

    def _concat_multi_view(self, obs_msg: Observation) -> Image.Image:
        """Concatenate left, center, and right camera images side-by-side.

        The three views are placed horizontally as [left | center | right],
        then resized back to the original single-image dimensions so the
        model's vision encoder receives the expected input resolution.
        """
        left = self._ros_img_to_pil(obs_msg.left_image)
        center = self._ros_img_to_pil(obs_msg.center_image)
        right = self._ros_img_to_pil(obs_msg.right_image)

        # Use center image dimensions as the reference size
        w, h = center.size

        # Resize left and right to match center dimensions (in case they differ)
        left = left.resize((w, h), Image.LANCZOS)
        right = right.resize((w, h), Image.LANCZOS)

        # Concatenate side-by-side: [left | center | right]
        concat = Image.new("RGB", (w * 3, h))
        concat.paste(left, (0, 0))
        concat.paste(center, (w, 0))
        concat.paste(right, (w * 2, 0))

        # Resize back to original single-image dimensions so the vision
        # encoder gets the resolution it expects (e.g. 224x224 after
        # processor transforms). This squeezes 3 views into 1 frame.
        concat = concat.resize((w, h), Image.LANCZOS)

        self.get_logger().info(
            f"Multi-view concat: 3x({w}x{h}) -> ({w}x{h})"
        )
        return concat

    def _build_task_prompt(self, task: Task) -> str:
        """Build a task-specific natural language instruction from the Task message.

        Uses plug_type, plug_name, port_name, and target_module_name to create
        a prompt tailored to each trial:
          - Trials 1 & 2: SFP module insertion into an SFP port on a NIC card
          - Trial 3: SC plug insertion into an SC port on the task board
        """
        # Normalize names for readability (e.g. "sfp_module" -> "SFP module")
        plug_display = task.plug_name.replace("_", " ")
        port_display = task.port_name.replace("_", " ")
        module_display = task.target_module_name.replace("_", " ")

        if task.plug_type == "sfp":
            # Trials 1 & 2: SFP module into SFP port on NIC card
            instruction = (
                f"insert the grasped {plug_display} into the"
                f"{port_display} on the {module_display}"
            )
        elif task.plug_type == "sc":
            # Trial 3: SC plug into SC port on task board
            instruction = (
                f"insert the grasped {plug_display} into the "
                f"{port_display} on the task board"
            )
        else:
            # Fallback for any unknown plug type
            instruction = (
                f"insert the grasped {plug_display} into the "
                f"{port_display} on the {module_display}"
            )

        self.get_logger().info(f"Task-specific instruction: {instruction}")
        return instruction

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info(f"RunOpenVLA.insert_cable() enter. Task: {task}")
        try:
            # Build a task-specific prompt from the Task message fields
            task_instruction = self._build_task_prompt(task)
            prompt = f"In: What action should the robot take to {task_instruction}?\nOut:"
            self.get_logger().info(f"Prompt: {prompt}")

            start_time = time.time()
            while time.time() - start_time < 30.0:
                loop_start = time.time()
                obs_msg = get_observation()
                if obs_msg is None:
                    self.get_logger().warn("No observation received")
                    continue

                pil_image = self._concat_multi_view(obs_msg)
                self.get_logger().info(f"Image size: {pil_image.size}, unnorm_key: {self.unnorm_key}")

                inputs = self.processor(prompt, pil_image).to(self.device, dtype=torch.bfloat16)
                self.get_logger().info(f"Input keys: {list(inputs.keys())}")

                with torch.inference_mode():
                    action = self.vla.predict_action(
                        **inputs,
                        unnorm_key=self.unnorm_key,
                        do_sample=False,
                    )
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
                send_feedback(f"openvla inference ({task.plug_type} insertion)...")

                # OpenVLA is slower (~1-3 Hz), adjust sleep accordingly
                elapsed = time.time() - loop_start
                time.sleep(max(0, 0.5 - elapsed))

        except Exception as e:
            import traceback
            self.get_logger().error(f"EXCEPTION in insert_cable: {e}")
            self.get_logger().error(traceback.format_exc())
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
