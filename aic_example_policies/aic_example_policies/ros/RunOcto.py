import time
import numpy as np
import torch
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Wrench

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode

from octo.model.octo_model_pt import OctoModelPt

# ===========================================================================
# Configuration
# ===========================================================================

# HuggingFace checkpoint ID (loads original JAX weights, converts to PyTorch)
OCTO_CHECKPOINT = "hf://rail-berkeley/octo-base-1.5"

# Dataset key for action unnormalization (UR5 data from Open X-Embodiment)
UNNORM_DATASET_KEY = "berkeley_autolab_ur5"

# Image sizes expected by Octo's pretrained vision encoders
PRIMARY_IMAGE_SIZE = 256  # image_primary
WRIST_IMAGE_SIZE = 128    # image_wrist

# How many future actions Octo predicts per inference call
ACTION_CHUNK_SIZE = 4

# Max time for one cable insertion attempt (seconds)
MAX_INSERTION_TIME = 60.0

# Time between executing each action step (seconds)
ACTION_STEP_INTERVAL = 0.05

# Scale factor on unnormalized actions before sending to controller
ACTION_SCALE = 1.0


class RunOcto(Policy):
    """Octo-PyTorch zero-shot baseline for AIC cable insertion."""

    def __init__(self, parent_node: Node):
        super().__init__(parent_node)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Loading Octo-PyTorch from {OCTO_CHECKPOINT}...")
        self.get_logger().info(f"Device: {self.device}")

        # Load JAX weights and convert to PyTorch in one call.
        # skip_keys_regex=".*hf_model" suppresses warnings about the T5
        # text encoder, which is loaded directly from HuggingFace instead.
        loaded = OctoModelPt.load_pretrained_from_jax(
            OCTO_CHECKPOINT, skip_keys_regex=".*hf_model"
        )
        self.model = loaded["octo_model"]
        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info("Octo-PyTorch loaded successfully.")

        # Resolve action unnormalization statistics
        self.unnorm_stats = self._resolve_unnorm_stats()

        # Observation history (Octo uses a window of 2 timesteps)
        self._primary_history: list[np.ndarray] = []
        self._wrist_history: list[np.ndarray] = []
        self._window_size = 2

        # Action chunk buffer
        self._action_chunk: list[np.ndarray] = []
        self._chunk_index: int = 0

        self.get_logger().info("Octo initialization complete.")
        self.get_logger().info("=" * 60)

    # ------------------------------------------------------------------
    # Unnormalization statistics
    # ------------------------------------------------------------------

    def _resolve_unnorm_stats(self) -> dict | None:
        """Find unnormalization statistics for UR5 actions.

        Without these, sample_actions() returns normalized values (~0 magnitude)
        and the robot won't move.
        """
        stats = getattr(self.model, "dataset_statistics", None)
        if stats is None:
            self.get_logger().warn("Model has no dataset_statistics attribute.")
            return None

        available = list(stats.keys())
        self.get_logger().info(f"Available dataset stats keys: {available}")

        # Try exact match
        if UNNORM_DATASET_KEY in stats:
            unnorm = stats[UNNORM_DATASET_KEY]["action"]
            self.get_logger().info(f"Using unnorm stats from '{UNNORM_DATASET_KEY}'")
            return unnorm

        # Fallback: any key containing "ur5"
        for key in available:
            if "ur5" in key.lower():
                unnorm = stats[key]["action"]
                self.get_logger().info(f"Using unnorm stats from fallback '{key}'")
                return unnorm

        self.get_logger().warn(
            f"No UR5 dataset found in stats! Actions will be normalized. "
            f"Available: {available}"
        )
        return None

    # ------------------------------------------------------------------
    # Image processing
    # ------------------------------------------------------------------

    def _ros_img_to_numpy(self, raw_img, target_size: int) -> np.ndarray:
        """Convert ROS Image to resized uint8 numpy array (H, W, 3)."""
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        if img_np.shape[0] != target_size or img_np.shape[1] != target_size:
            img_np = cv2.resize(
                img_np, (target_size, target_size), interpolation=cv2.INTER_AREA
            )
        return img_np

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------

    def _build_observation(self, obs_msg: Observation) -> dict:
        """Build Octo observation dict from ROS Observation.

        Uses center camera as image_primary (256x256) and
        left camera as image_wrist (128x128).
        Right camera is unused — Octo only supports 2 image inputs.
        """
        primary = self._ros_img_to_numpy(obs_msg.center_image, PRIMARY_IMAGE_SIZE)
        wrist = self._ros_img_to_numpy(obs_msg.left_image, WRIST_IMAGE_SIZE)

        self._primary_history.append(primary)
        self._wrist_history.append(wrist)

        if len(self._primary_history) > self._window_size:
            self._primary_history = self._primary_history[-self._window_size:]
            self._wrist_history = self._wrist_history[-self._window_size:]

        # Pad if window not full yet
        n = len(self._primary_history)
        pad_mask = [True] * n

        primary_imgs = list(self._primary_history)
        wrist_imgs = list(self._wrist_history)

        while len(primary_imgs) < self._window_size:
            primary_imgs.insert(0, primary_imgs[0])
            wrist_imgs.insert(0, wrist_imgs[0])
            pad_mask.insert(0, False)

        observation = {
            "image_primary": np.stack(primary_imgs)[np.newaxis],  # (1, 2, 256, 256, 3)
            "image_wrist": np.stack(wrist_imgs)[np.newaxis],      # (1, 2, 128, 128, 3)
            "timestep_pad_mask": np.array([pad_mask]),             # (1, 2)
            "pad_mask_dict": {
                "image_primary": np.array([[True, True]]),
                "image_wrist": np.array([[True, True]]),
            },
        }
        return observation

    # ------------------------------------------------------------------
    # Task prompt
    # ------------------------------------------------------------------

    def _build_task_prompt(self, task: Task) -> str:
        """Build language instruction from the Task message."""
        plug = task.plug_name.replace("_", " ")
        port = task.port_name.replace("_", " ")
        module = task.target_module_name.replace("_", " ")

        if task.plug_type == "sfp":
            return f"insert the {plug} into the {port} on the {module}"
        elif task.plug_type == "sc":
            return f"insert the {plug} into the {port} on the task board"
        else:
            return f"insert the {plug} into the {port} on the {module}"

    # ------------------------------------------------------------------
    # Core policy loop
    # ------------------------------------------------------------------

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info(
            f"RunOcto.insert_cable() enter. "
            f"plug_type={task.plug_type}, plug={task.plug_name}, "
            f"port={task.port_name}, module={task.target_module_name}"
        )

        # Reset state
        self._primary_history = []
        self._wrist_history = []
        self._action_chunk = []
        self._chunk_index = 0

        # Build language task (only once per insertion)
        instruction = self._build_task_prompt(task)
        self.get_logger().info(f"Task instruction: '{instruction}'")
        task_spec = self.model.create_tasks(texts=[instruction])

        start_time = time.time()
        step_count = 0

        try:
            while time.time() - start_time < MAX_INSERTION_TIME:
                loop_start = time.time()

                # Re-plan when chunk is exhausted
                if self._chunk_index >= len(self._action_chunk):
                    obs_msg = get_observation()
                    if obs_msg is None:
                        self.get_logger().warn("No observation, retrying...")
                        time.sleep(0.01)
                        continue

                    observation = self._build_observation(obs_msg)

                    # Run inference
                    inference_start = time.time()
                    with torch.no_grad():
                        raw_actions = self.model.sample_actions(
                            observation,
                            task_spec,
                            unnormalization_statistics=self.unnorm_stats,
                        )
                    inference_time = time.time() - inference_start

                    # raw_actions: (batch=1, chunk=4, action_dim=7)
                    if isinstance(raw_actions, torch.Tensor):
                        chunk = raw_actions[0].cpu().numpy()
                    else:
                        chunk = np.array(raw_actions[0])

                    self._action_chunk = [chunk[i] for i in range(chunk.shape[0])]
                    self._chunk_index = 0

                    self.get_logger().info(
                        f"Octo inference: {len(self._action_chunk)} actions "
                        f"in {inference_time:.3f}s "
                        f"({len(self._action_chunk) / max(inference_time, 1e-6):.1f} Hz). "
                        f"First: {self._action_chunk[0]}"
                    )

                # Execute next action from chunk
                action = self._action_chunk[self._chunk_index] * ACTION_SCALE
                self._chunk_index += 1

                # Octo 7-DOF: [dx, dy, dz, drx, dry, drz, gripper]
                twist = Twist(
                    linear=Vector3(
                        x=float(action[0]),
                        y=float(action[1]),
                        z=float(action[2]),
                    ),
                    angular=Vector3(
                        x=float(action[3]),
                        y=float(action[4]),
                        z=float(action[5]),
                    ),
                )

                move_robot(motion_update=self._make_velocity_command(twist))

                step_count += 1
                if step_count % ACTION_CHUNK_SIZE == 0:
                    send_feedback(
                        f"Octo: step {step_count}, "
                        f"chunk {step_count // ACTION_CHUNK_SIZE}, "
                        f"{task.plug_type} insertion"
                    )

                elapsed = time.time() - loop_start
                time.sleep(max(0, ACTION_STEP_INTERVAL - elapsed))

        except Exception as e:
            import traceback
            self.get_logger().error(f"EXCEPTION: {e}")
            self.get_logger().error(traceback.format_exc())

        self.get_logger().info(
            f"RunOcto.insert_cable() exit after {step_count} steps "
            f"({time.time() - start_time:.1f}s)"
        )
        return True

    # ------------------------------------------------------------------
    # Motion command
    # ------------------------------------------------------------------

    def _make_velocity_command(
        self, twist: Twist, frame_id: str = "base_link"
    ) -> MotionUpdate:
        msg = MotionUpdate()
        msg.velocity = twist
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_stiffness = np.diag(
            [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
        ).flatten().tolist()
        msg.target_damping = np.diag(
            [40.0, 40.0, 40.0, 15.0, 15.0, 15.0]
        ).flatten().tolist()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        return msg
