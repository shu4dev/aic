import time
import numpy as np
import jax
import jax.numpy as jnp
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

from octo.model.octo_model import OctoModel

# ===========================================================================
# Configuration
# ===========================================================================

# HuggingFace checkpoint ID for the original JAX Octo
OCTO_CHECKPOINT = "hf://rail-berkeley/octo-base-1.5"

# Dataset key for action unnormalization.
# berkeley_autolab_ur5 is the closest match to the AIC UR5e setup in the
# Open X-Embodiment pretraining mix. If it is absent from the checkpoint's
# dataset_statistics, the code falls back to bridge_dataset and finally to
# None (normalized-only actions, robot will barely move — useful for
# verifying the pipeline is connected before switching to a real key).
UNNORM_DATASET_KEY = "berkeley_autolab_ur5"
UNNORM_FALLBACK_KEY = "bridge_dataset"

# Image sizes expected by Octo's pretrained vision encoders
PRIMARY_IMAGE_SIZE = 256   # image_primary  (center camera → 256×256)
WRIST_IMAGE_SIZE   = 128   # image_wrist    (left camera   → 128×128)

# Octo window size (number of consecutive timesteps fed to the transformer)
WINDOW_SIZE = 2

# Octo predicts ACTION_CHUNK_SIZE future actions per inference call.
# We execute all of them before calling inference again (action chunking).
ACTION_CHUNK_SIZE = 4

# Maximum wall-clock time for one cable-insertion attempt (seconds)
MAX_INSERTION_TIME = 60.0

# Target control frequency (seconds per action step)
ACTION_STEP_INTERVAL = 0.05   # → ~20 Hz

# Multiplicative scale applied to unnormalized Cartesian delta actions.
# Start at 1.0 and reduce if the robot moves too aggressively.
ACTION_SCALE = 1.0

# Impedance controller gains sent with every MotionUpdate
STIFFNESS = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]   # 6-element diagonal
DAMPING    = [40.0,  40.0,  40.0, 15.0, 15.0, 15.0]   # 6-element diagonal
WRENCH_FEEDBACK_GAINS = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]


class RunOcto(Policy):
    """Zero-shot Octo baseline (original JAX implementation) for AIC cable insertion.

    Run with:
        pixi run ros2 run aic_model aic_model \\
            --ros-args -p use_sim_time:=true \\
                       -p policy:=aic_example_policies.ros.RunOcto
    """

    # ------------------------------------------------------------------
    # Construction & model loading
    # ------------------------------------------------------------------

    def __init__(self, parent_node: Node):
        super().__init__(parent_node)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Loading Octo (JAX) from {OCTO_CHECKPOINT} …")

        # OctoModel.load_pretrained() returns the model directly — no dict wrapping.
        # JAX handles device placement automatically; no .to(device) needed.
        self.model: OctoModel = OctoModel.load_pretrained(OCTO_CHECKPOINT)

        self.get_logger().info("Octo model loaded successfully.")
        self.get_logger().info(self.model.get_pretty_spec())

        # Resolve unnormalization statistics once at startup
        self.unnorm_stats = self._resolve_unnorm_stats()

        # Per-episode observation history (length ≤ WINDOW_SIZE)
        self._primary_history: list[np.ndarray] = []
        self._wrist_history:   list[np.ndarray] = []

        # Action-chunk buffer: refilled every ACTION_CHUNK_SIZE steps
        self._action_chunk: list[np.ndarray] = []
        self._chunk_index:  int = 0

        # JAX RNG key — split at every inference call so we get different samples
        self._rng = jax.random.PRNGKey(0)

        self.get_logger().info("RunOcto initialisation complete.")
        self.get_logger().info("=" * 60)

    # ------------------------------------------------------------------
    # Unnormalization statistics
    # ------------------------------------------------------------------

    def _resolve_unnorm_stats(self) -> dict | None:
        """Select action unnormalization statistics from the checkpoint.

        The Octo checkpoint embeds per-dataset mean/std for actions.
        Without correct statistics, sample_actions() returns values close
        to zero and the robot will not move meaningfully.
        """
        stats = getattr(self.model, "dataset_statistics", None)
        if stats is None:
            self.get_logger().warn(
                "Checkpoint has no dataset_statistics. "
                "Actions will remain in normalized space (robot may not move)."
            )
            return None

        available = list(stats.keys())
        self.get_logger().info(f"Available dataset_statistics keys: {available}")

        # 1. Exact preferred key
        if UNNORM_DATASET_KEY in stats:
            self.get_logger().info(
                f"Using unnorm stats: '{UNNORM_DATASET_KEY}'"
            )
            return stats[UNNORM_DATASET_KEY]["action"]

        # 2. Any key containing "ur5" (case-insensitive)
        for key in available:
            if "ur5" in key.lower():
                self.get_logger().info(
                    f"Preferred key not found; using fallback '{key}' (contains 'ur5')"
                )
                return stats[key]["action"]

        # 3. bridge_dataset as a final fallback (action scale is roughly similar)
        if UNNORM_FALLBACK_KEY in stats:
            self.get_logger().warn(
                f"No UR5 stats found. Using '{UNNORM_FALLBACK_KEY}' as fallback. "
                "Action magnitudes will be approximate — tune ACTION_SCALE accordingly."
            )
            return stats[UNNORM_FALLBACK_KEY]["action"]

        self.get_logger().warn(
            "No suitable unnorm key found. "
            f"Available: {available}. Robot may barely move."
        )
        return None

    # ------------------------------------------------------------------
    # Image processing
    # ------------------------------------------------------------------

    def _ros_img_to_numpy(self, raw_img, target_size: int) -> np.ndarray:
        """Decode a ROS Image message to a uint8 numpy array (H, W, 3).

        Assumes the image is already in RGB encoding (the AIC adapter
        publishes uncompressed RGB frames).  Resizes to target_size×target_size
        using INTER_AREA (best quality for downsampling).
        """
        img = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        if img.shape[0] != target_size or img.shape[1] != target_size:
            img = cv2.resize(img, (target_size, target_size),
                             interpolation=cv2.INTER_AREA)
        return img  # (H, W, 3) uint8

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------

    def _build_observation(self, obs_msg: Observation) -> dict:
        """Construct the observation dict expected by OctoModel.sample_actions().

        Shapes after this function:
            image_primary      : (1, WINDOW_SIZE, 256, 256, 3)  uint8
            image_wrist        : (1, WINDOW_SIZE, 128, 128, 3)  uint8
            timestep_pad_mask  : (1, WINDOW_SIZE)                bool
        """
        # Decode images from the three wrist cameras.
        # center_image → image_primary (256×256)
        # left_image   → image_wrist   (128×128)
        # right_image is unused — Octo supports at most two image inputs.
        primary = self._ros_img_to_numpy(obs_msg.center_image, PRIMARY_IMAGE_SIZE)
        wrist   = self._ros_img_to_numpy(obs_msg.left_image,   WRIST_IMAGE_SIZE)

        # Append to rolling history, keeping at most WINDOW_SIZE entries
        self._primary_history.append(primary)
        self._wrist_history.append(wrist)
        if len(self._primary_history) > WINDOW_SIZE:
            self._primary_history = self._primary_history[-WINDOW_SIZE:]
            self._wrist_history   = self._wrist_history[-WINDOW_SIZE:]

        n = len(self._primary_history)

        # Build pad mask: False for padding frames, True for real observations.
        # Octo's convention: False = padding (should be ignored by attention).
        pad_mask = np.array([[False] * (WINDOW_SIZE - n) + [True] * n])  # (1, W)

        # Repeat the earliest frame to fill the window when history is short
        primary_imgs = (
            [self._primary_history[0]] * (WINDOW_SIZE - n) + list(self._primary_history)
        )
        wrist_imgs = (
            [self._wrist_history[0]] * (WINDOW_SIZE - n) + list(self._wrist_history)
        )

        observation = {
            # (1, WINDOW_SIZE, H, W, 3)
            "image_primary":     np.stack(primary_imgs)[np.newaxis],
            "image_wrist":       np.stack(wrist_imgs)[np.newaxis],
            # (1, WINDOW_SIZE)  — tells Octo which timesteps are real
            "timestep_pad_mask": pad_mask,
        }
        return observation

    # ------------------------------------------------------------------
    # Task prompt
    # ------------------------------------------------------------------

    def _build_task_prompt(self, task: Task) -> str:
        """Map the Task message fields to a natural-language instruction string."""
        plug   = task.plug_name.replace("_", " ")
        port   = task.port_name.replace("_", " ")
        module = task.target_module_name.replace("_", " ")

        if task.plug_type == "sfp":
            return f"insert the {plug} SFP into the {port} on the {module}"
        elif task.plug_type == "sc":
            return f"insert the {plug} connector into the {port} on the task board"
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
    ) -> bool:
        self.get_logger().info(
            f"RunOcto.insert_cable() — "
            f"plug_type={task.plug_type!r}  plug={task.plug_name!r}  "
            f"port={task.port_name!r}  module={task.target_module_name!r}"
        )

        # ── Reset per-episode state ──────────────────────────────────
        self._primary_history = []
        self._wrist_history   = []
        self._action_chunk    = []
        self._chunk_index     = 0

        # ── Build language task spec (created once per insertion) ────
        instruction = self._build_task_prompt(task)
        self.get_logger().info(f"Language instruction: '{instruction}'")
        # create_tasks returns a dict consumed by sample_actions()
        task_spec = self.model.create_tasks(texts=[instruction])

        start_time = time.time()
        step_count = 0

        try:
            while time.time() - start_time < MAX_INSERTION_TIME:
                loop_start = time.time()

                # ── Refill action chunk when exhausted ──────────────
                if self._chunk_index >= len(self._action_chunk):
                    obs_msg = get_observation()
                    if obs_msg is None:
                        self.get_logger().warn("get_observation() returned None — retrying.")
                        time.sleep(0.01)
                        continue

                    observation = self._build_observation(obs_msg)

                    # Split RNG so each call produces a different sample
                    self._rng, inference_rng = jax.random.split(self._rng)

                    t_infer = time.time()
                    # sample_actions() is a JAX function; it returns a JAX
                    # array on first call (may trigger JIT compilation) and
                    # a plain numpy-compatible array on subsequent calls.
                    # Shape: (batch=1, action_horizon=4, action_dim=7)
                    raw_actions = self.model.sample_actions(
                        observation,
                        task_spec,
                        unnormalization_statistics=self.unnorm_stats,
                        rng=inference_rng,
                    )
                    infer_ms = (time.time() - t_infer) * 1000

                    # Convert to numpy if still a JAX array
                    chunk = np.array(raw_actions[0])  # (ACTION_CHUNK_SIZE, 7)

                    self._action_chunk = [chunk[i] for i in range(chunk.shape[0])]
                    self._chunk_index  = 0

                    self.get_logger().info(
                        f"Inference: {len(self._action_chunk)} actions "
                        f"in {infer_ms:.1f} ms  |  "
                        f"action[0]={self._action_chunk[0]}"
                    )

                # ── Execute next action ──────────────────────────────
                action = self._action_chunk[self._chunk_index] * ACTION_SCALE
                self._chunk_index += 1

                # Octo outputs 7-DOF end-effector delta:
                #   [dx, dy, dz, drx, dry, drz, gripper]
                # The AIC controller interprets Twist.linear as [dx, dy, dz]
                # and Twist.angular as [drx, dry, drz] in the base frame.
                twist = Twist(
                    linear  = Vector3(x=float(action[0]),
                                      y=float(action[1]),
                                      z=float(action[2])),
                    angular = Vector3(x=float(action[3]),
                                      y=float(action[4]),
                                      z=float(action[5])),
                )
                # Gripper: Octo convention is 0 = open, 1 = closed.
                # Map to the AIC gripper range [0, 1] directly.
                # (Invert here if your gripper driver expects the opposite.)
                gripper_cmd = float(np.clip(action[6], 0.0, 1.0))

                motion_cmd = self._make_velocity_command(twist, gripper=gripper_cmd)
                move_robot(motion_update=motion_cmd)

                step_count += 1
                if step_count % ACTION_CHUNK_SIZE == 0:
                    elapsed = time.time() - start_time
                    send_feedback(
                        f"Octo: step={step_count}  chunk={step_count // ACTION_CHUNK_SIZE}"
                        f"  elapsed={elapsed:.1f}s  plug={task.plug_type}"
                    )

                # ── Rate-limit to ACTION_STEP_INTERVAL ──────────────
                sleep_time = ACTION_STEP_INTERVAL - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception:
            import traceback
            self.get_logger().error(
                f"Exception in RunOcto.insert_cable():\n{traceback.format_exc()}"
            )

        self.get_logger().info(
            f"RunOcto.insert_cable() finished — "
            f"{step_count} steps in {time.time() - start_time:.1f}s"
        )
        return True

    # ------------------------------------------------------------------
    # Motion command builder
    # ------------------------------------------------------------------

    def _make_velocity_command(
        self,
        twist: Twist,
        gripper: float = 0.0,
        frame_id: str = "base_link",
    ) -> MotionUpdate:
        """Wrap a Twist into the MotionUpdate message expected by the AIC controller."""
        msg = MotionUpdate()

        msg.header.frame_id = frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.velocity = twist

        # Gripper command (range [0, 1]; 0 = fully open, 1 = fully closed)
        msg.gripper_command = gripper

        # Diagonal impedance gains as plain 6-element lists (NOT a 6×6 matrix)
        msg.target_stiffness = STIFFNESS
        msg.target_damping   = DAMPING

        # No active wrench feedforward for zero-shot baseline
        msg.feedforward_wrench_at_tip = Wrench(
            force  = Vector3(x=0.0, y=0.0, z=0.0),
            torque = Vector3(x=0.0, y=0.0, z=0.0),
        )

        msg.wrench_feedback_gains_at_tip = WRENCH_FEEDBACK_GAINS

        msg.trajectory_generation_mode.mode = (
            TrajectoryGenerationMode.MODE_VELOCITY
        )

        return msg
