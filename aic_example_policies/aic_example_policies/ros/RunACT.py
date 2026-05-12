#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

from __future__ import annotations  # lazy annotation evaluation — required so
# `torch.device` and other heavy-module type hints don't trip the deferred
# import (heavy modules are loaded later via _import_heavy()).

import os

os.environ["HF_HUB_ENABLE_HF_TRANSFER"] = "1"

import time
import json
import numpy as np
import draccus
from pathlib import Path
from typing import Dict, TYPE_CHECKING
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Wrench

from aic_example_policies.utils.wrench_smoother import (
    DEFAULT_MODE as WRENCH_DEFAULT_MODE,
    DEFAULT_WINDOW_S as WRENCH_DEFAULT_WINDOW_S,
    WrenchSmoother,
)
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task

from aic_control_interfaces.msg import (
    JointMotionUpdate,
    MotionUpdate,
    TrajectoryGenerationMode,
)
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

# Heavy imports (torch, cv2, lerobot, safetensors, huggingface_hub) are
# deferred to RunACT.__init__() via _import_heavy() so this module imports in
# <1 s. That lets AicModel.__init__ return quickly and rclpy's executor start
# spinning in time for aic_engine's discovery + configure handshake.
if TYPE_CHECKING:
    import torch
    import cv2
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.act.configuration_act import ACTConfig
    from safetensors.torch import load_file
    from huggingface_hub import snapshot_download


def _import_heavy():
    """Populate torch/cv2/ACTPolicy/ACTConfig/load_file/snapshot_download as
    module globals on first call. The rest of this module references them
    by bare name, so once imported they behave like top-level imports."""
    global torch, cv2, ACTPolicy, ACTConfig, load_file, snapshot_download
    import torch as _torch
    import cv2 as _cv2
    from lerobot.policies.act.modeling_act import ACTPolicy as _ACTPolicy
    from lerobot.policies.act.configuration_act import ACTConfig as _ACTConfig
    from safetensors.torch import load_file as _load_file
    from huggingface_hub import snapshot_download as _snapshot_download
    torch = _torch
    cv2 = _cv2
    ACTPolicy = _ACTPolicy
    ACTConfig = _ACTConfig
    load_file = _load_file
    snapshot_download = _snapshot_download


def _pose_to_xyz_rot6d(x, y, z, qx, qy, qz, qw):
    """Quaternion pose → 9D [x,y,z, r1(3), r2(3)] (first two rot-matrix columns).

    Mirrors mcap_to_lerobot.pose_to_xyz_rot6d — must stay in sync since
    training normalization stats are computed against that encoding.
    """
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    r1x = 1 - 2*(yy + zz); r1y = 2*(xy + wz); r1z = 2*(xz - wy)
    r2x = 2*(xy - wz); r2y = 1 - 2*(xx + zz); r2z = 2*(yz + wx)
    return [x, y, z, r1x, r1y, r1z, r2x, r2y, r2z]


def _rot6d_to_quat_xyzw(r1, r2):
    """Decode 6D rotation (two column vectors) → [qx,qy,qz,qw].

    Uses Gram-Schmidt to orthonormalize, then stable matrix-to-quaternion.
    """
    r1 = np.asarray(r1, dtype=np.float64)
    r2 = np.asarray(r2, dtype=np.float64)
    n1 = np.linalg.norm(r1)
    b1 = r1 / n1 if n1 > 1e-10 else np.array([1.0, 0.0, 0.0])
    r2_ortho = r2 - np.dot(b1, r2) * b1
    n2 = np.linalg.norm(r2_ortho)
    b2 = r2_ortho / n2 if n2 > 1e-10 else np.array([0.0, 1.0, 0.0])
    b3 = np.cross(b1, b2)
    R = np.column_stack([b1, b2, b3])
    # Shepperd's stable matrix-to-quaternion
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


# UR-style joint order, used to extract the 6-dof joint vector from an
# Observation message (which has positions in JointState name-order).
_UR_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def _joint_positions_from_obs(obs_msg):
    """Extract the 6-dof joint position vector from an Observation message."""
    name_to_pos = dict(zip(obs_msg.joint_states.name, obs_msg.joint_states.position))
    return [name_to_pos.get(jn, 0.0) for jn in _UR_JOINT_NAMES]


def _quaternion_slerp_xyzw(q0, q1, t):
    """Slerp from q0 to q1 with t in [0, 1]. Inputs are [x, y, z, w] arrays.

    Used by _act_step's cartesian_pose mode to interpolate the gripper
    orientation toward the predicted target by ACT_SPEED_FACTOR.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = float(np.dot(q0, q1))
    # Shortest-path: flip q1 if it's on the opposite hemisphere.
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    # Near-parallel: lerp + renormalize is numerically safer than slerp.
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
        return out / np.linalg.norm(out)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0
    return s0 * q0 + s1 * q1


class RunACT(Policy):
    """ACT (Action Chunking Transformer) inference policy for cable insertion.

    Key design decisions:

    1. **Two action modes: joint (preferred) or Cartesian twist deltas.**
       - Joint mode: model predicts absolute joint positions [q1..q6],
         sent directly via JointMotionUpdate. Simpler, no IK, no quaternion
         issues. Auto-detected from action feature names in config.json.
       - Cartesian mode (legacy): model predicts 6D twist deltas
         [dx, dy, dz, dax, day, daz], applied as current_tcp + delta via
         MODE_POSITION MotionUpdate.

    2. **Temporal ensembling (on by default, coeff=0.01).**
       The model runs every timestep and overlapping chunk predictions are
       exponentially averaged (coeff=0.01 from the ACT paper). Set
       ACT_TEMPORAL_ENSEMBLE=off to disable and use RESET_EVERY instead.
       Set ACT_RESET_EVERY to control chunk reset interval when ensembling
       is off (default: 10 = re-predict every 0.5s).

    3. **MODE_POSITION with relative deltas, not MODE_VELOCITY.**
       Although the model outputs velocity-like deltas, we convert each action
       to an absolute pose target (current_tcp + delta) and send it via
       MODE_POSITION. We do NOT use MODE_VELOCITY because the underlying
       impedance controller integrates velocity commands into an internal
       position target. If the robot can't track fast enough (finite stiffness),
       that internal target outruns the actual TCP. Once the target escapes the
       robot's reachable workspace, the controller can't generate sufficient
       joint torques, and the robot mysteriously stops moving — even though
       velocity commands are still being sent. By applying each delta relative
       to the current TCP and commanding in MODE_POSITION, the position target
       never drifts more than one timestep ahead of reality.
    """

    def __init__(self, parent_node: Node):
        super().__init__(parent_node)
        # Trigger the deferred heavy imports (torch, cv2, lerobot, ...) — only
        # at policy-instantiation time, not at module import. This is the
        # critical step that lets aic_model_node start spinning fast enough
        # for aic_engine to find it.
        _import_heavy()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # Per-trial cache for the port_entrance pose in base_link (static per trial).
        # Used only in "joints_port" state mode.
        self._port_entrance_baselink = None

        # Wrench smoother — same window/mode as collection-time CheatCode and
        # offline mcap_to_lerobot, so the wrench distribution the model sees
        # at eval matches the dataset it was trained on. Subscribes to
        # /fts_broadcaster/wrench directly (faster than the camera-rate-limited
        # Observation aggregation in aic_adapter).
        # ReentrantCallbackGroup so the wrench sub keeps draining while the
        # policy timer holds the default MutuallyExclusive group during GPU
        # forward (~50–100 ms). Without this, the smoothed-wrench window goes
        # stale at predict() time → the 21D obs the model sees diverges from
        # training distribution. (Same fix as ImprovedCheatCode.py.)
        self._wrench_cb_group = ReentrantCallbackGroup()
        self._wrench_smoother = WrenchSmoother(
            parent_node, callback_group=self._wrench_cb_group
        )
        self._wrench_window_s = float(
            os.environ.get("WRENCH_SMOOTH_WINDOW_S", str(WRENCH_DEFAULT_WINDOW_S))
        )
        self._wrench_mode = os.environ.get("WRENCH_SMOOTH_MODE", WRENCH_DEFAULT_MODE)

        # -------------------------------------------------------------------------
        # 1. Configuration & Weights Loading
        # -------------------------------------------------------------------------
        policy_path_env = os.environ.get("ACT_POLICY_PATH", "")
        if policy_path_env:
            policy_path = Path(policy_path_env)
        else:
            repo_id = "grkw/aic_act_policy"
            policy_path = Path(
                snapshot_download(
                    repo_id=repo_id,
                    allow_patterns=["config.json", "model.safetensors", "*.safetensors"],
                )
            )

        # Load Config Manually (Fixes 'Draccus' error by removing unknown 'type' field)
        with open(policy_path / "config.json", "r") as f:
            config_dict = json.load(f)
            if "type" in config_dict:
                del config_dict["type"]

        config = draccus.decode(ACTConfig, config_dict)

        # Load Policy Architecture & Weights
        self.policy = ACTPolicy(config)
        model_weights_path = policy_path / "model.safetensors"
        self.policy.load_state_dict(load_file(model_weights_path))
        self.policy.eval()
        self.policy.to(self.device)

        self.get_logger().info(f"ACT Policy loaded on {self.device} from {policy_path}")

        # -------------------------------------------------------------------------
        # 2. Normalization Stats Loading
        # -------------------------------------------------------------------------
        stats_path = (
            policy_path / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        )
        stats = load_file(stats_path)

        # Helper to extract and shape stats for broadcasting
        def get_stat(key, shape):
            return stats[key].to(self.device).view(*shape)

        # Image Stats (1, 3, 1, 1) for broadcasting against (Batch, Channel, Height, Width)
        self.img_stats = {
            "left": {
                "mean": get_stat("observation.images.left_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.left_camera.std", (1, 3, 1, 1)),
            },
            "center": {
                "mean": get_stat("observation.images.center_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.center_camera.std", (1, 3, 1, 1)),
            },
            "right": {
                "mean": get_stat("observation.images.right_camera.mean", (1, 3, 1, 1)),
                "std": get_stat("observation.images.right_camera.std", (1, 3, 1, 1)),
            },
        }
        # Robot State Stats
        self.state_mean = get_stat("observation.state.mean", (1, -1))
        self.state_std = get_stat("observation.state.std", (1, -1))
        state_dim = int(self.state_mean.shape[-1])
        # Auto-detect mode from state dimensionality:
        #   6D   = joints-only
        #   9D   = joints + port_entrance (ground-truth target) diagnostic
        #   15D  = joints + 9D TCP pose (xyz + 6D rotation)
        #   25D+ = full proprio (+wrench = 31D)
        if state_dim == 6:
            self.obs_state_mode = "joints"
        elif state_dim == 9:
            self.obs_state_mode = "joints_port"
        elif state_dim == 15:
            self.obs_state_mode = "joints_pose"
        elif state_dim == 21:
            self.obs_state_mode = "joints_pose_wrench"
        else:
            self.obs_state_mode = "full"
        self.get_logger().info(
            f"Observation state mode: {self.obs_state_mode} (dim={state_dim})"
        )

        # Action Stats - Used for Un-normalization
        # Auto-detect normalization method from saved config
        self.action_norm_mode = config_dict.get(
            "normalization_mapping", {}
        ).get("ACTION", "MEAN_STD")
        if self.action_norm_mode == "MINMAX":
            self.action_min = get_stat("action.min", (1, -1))
            self.action_max = get_stat("action.max", (1, -1))
            self.get_logger().info("Action normalization: MINMAX")
        else:
            self.action_mean = get_stat("action.mean", (1, -1))
            self.action_std = get_stat("action.std", (1, -1))
            self.get_logger().info("Action normalization: MEAN_STD")

        # Detect action mode from dimensionality + normalization magnitudes:
        #   9D → cartesian_pose (absolute xyz + 6D rotation)
        #   6D, |mean|_max > 0.1 → joint (abs joint positions, ~1-2 rad)
        #   6D, |mean|_max small → cartesian (twist deltas, ~0 mean)
        action_dim = int(self.action_mean.shape[-1]) if self.action_norm_mode != "MINMAX" else int(self.action_min.shape[-1])
        if action_dim == 9:
            self.action_mode = "cartesian_pose"
        else:
            action_mean_mag = float(stats["action.mean"].abs().max())
            self.action_mode = "joint" if action_mean_mag > 0.1 else "cartesian"
        self.get_logger().info(
            f"Action mode: {self.action_mode} (dim={action_dim})"
        )

        # Config
        # image_scaling resizes sim's 1152x1024 camera output to match training
        # data resolution. ACT_IMAGE_SCALING env var overrides at eval time so a
        # half-resolution-trained policy (exp29-halfres) can be evaluated without
        # editing this file: ACT_IMAGE_SCALING=0.125 bash scripts/eval_act.sh ...
        # Default 0.25 → 288x256, matching the v3 (exp29-clean) training resolution.
        self.image_scaling = float(os.environ.get("ACT_IMAGE_SCALING", "0.25"))

        # Per-step motion scale applied in _act_step before dispatch.
        # 1.0 reproduces the model's commanded action verbatim. <1.0 slows
        # all motion uniformly by interpolating from current state toward
        # the predicted target (for absolute-target modes) or by scaling
        # the delta directly (for twist-delta mode). 0.0 freezes the robot.
        # Values >1.0 amplify and are off-distribution.
        self._speed_factor = float(os.environ.get("ACT_SPEED_FACTOR", "1.0"))
        if self._speed_factor != 1.0:
            self.get_logger().info(
                f"[ACT] ACT_SPEED_FACTOR={self._speed_factor} "
                f"(per-step motion scaling enabled)"
            )

        # Approach-quality eval. When EVAL_GT_LOG=1 the policy looks up the
        # port + plug TFs at runtime and logs a multi-component alignment
        # report ([eval] xy=... z=... tilt=... dist=... READY/NOT_READY).
        # Requires launching with ground_truth:=true so the scoring TFs are
        # relayed to /tf. Off by default — production runs unchanged.
        self._eval_gt_log = os.environ.get("EVAL_GT_LOG", "0") == "1"
        self._eval_xy_ok_m = float(os.environ.get("EVAL_XY_OK_MM", "1.5")) / 1000.0
        self._eval_z_ok_m = float(os.environ.get("EVAL_Z_OK_MM", "10.0")) / 1000.0
        self._eval_tilt_ok_deg = float(os.environ.get("EVAL_TILT_OK_DEG", "5.0"))
        self._eval_tf_warned = False
        if self._eval_gt_log:
            self.get_logger().info(
                f"[ACT] EVAL_GT_LOG=1 "
                f"(xy_ok={self._eval_xy_ok_m*1000:.1f}mm "
                f"z_ok={self._eval_z_ok_m*1000:.1f}mm "
                f"tilt_ok={self._eval_tilt_ok_deg:.1f}°) "
                f"— requires ground_truth:=true at launch"
            )

        self.get_logger().info("Normalization statistics loaded successfully.")

    @staticmethod
    def _img_to_tensor(
        raw_img,
        device: torch.device,
        scale: float,
        mean: torch.Tensor,
        std: torch.Tensor,
    ) -> torch.Tensor:
        """Converts ROS Image -> RGB -> Resized -> Permuted -> Normalized Tensor.

        Training data is loaded via PyAV `to_ndarray(format="rgb24")`, so the
        model sees RGB. Sim cameras publish rgb8 directly, so no conversion
        needed here — just reshape the raw bytes.
        """
        # 1. Bytes to Numpy (H, W, C) in RGB order (matches training)
        enc = raw_img.encoding.lower() if raw_img.encoding else "rgb8"
        h, w = raw_img.height, raw_img.width
        if enc in ("rgb8",):
            img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 3)
        elif enc in ("bgr8",):
            raw = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 3)
            img_np = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
        elif enc in ("rgba8",):
            raw = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 4)
            img_np = cv2.cvtColor(raw, cv2.COLOR_RGBA2RGB)
        elif enc in ("bgra8",):
            raw = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 4)
            img_np = cv2.cvtColor(raw, cv2.COLOR_BGRA2RGB)
        else:
            # Unknown — assume rgb8
            img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 3)

        # 2. Resize
        if scale != 1.0:
            img_np = cv2.resize(
                img_np, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )

        # 3. To Tensor -> Permute (HWC -> CHW) -> Float -> Div(255) -> Batch Dim
        tensor = (
            torch.from_numpy(img_np)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(device)
        )

        # 4. Normalize (Apply Mean/Std)
        # Formula: (x - mean) / std
        return (tensor - mean) / std

    def prepare_observations(self, obs_msg: Observation) -> Dict[str, torch.Tensor]:
        """Convert ROS Observation message into dictionary of normalized tensors."""

        # --- Process Cameras ---
        obs = {
            "observation.images.left_camera": self._img_to_tensor(
                obs_msg.left_image,
                self.device,
                self.image_scaling,
                self.img_stats["left"]["mean"],
                self.img_stats["left"]["std"],
            ),
            "observation.images.center_camera": self._img_to_tensor(
                obs_msg.center_image,
                self.device,
                self.image_scaling,
                self.img_stats["center"]["mean"],
                self.img_stats["center"]["std"],
            ),
            "observation.images.right_camera": self._img_to_tensor(
                obs_msg.right_image,
                self.device,
                self.image_scaling,
                self.img_stats["right"]["mean"],
                self.img_stats["right"]["std"],
            ),
        }

        # One-shot diagnostic: verify per-camera tensor shape + pixel stats match training.
        # Training expects [1, 3, 256, 288] normalized against ImageNet stats.
        # Pre-normalization pixel values should be in [0, 1]; post-normalization
        # should be roughly centered around 0 with std ~1.
        if not hasattr(self, "_logged_image_encoding"):
            left_t = obs["observation.images.left_camera"]
            self.get_logger().info(
                f"[ACT] Camera encodings — left={obs_msg.left_image.encoding!r} "
                f"center={obs_msg.center_image.encoding!r} "
                f"right={obs_msg.right_image.encoding!r} "
                f"(raw {obs_msg.left_image.width}x{obs_msg.left_image.height}, "
                f"scale={self.image_scaling})"
            )
            self.get_logger().info(
                f"[ACT] Tensor shape: {tuple(left_t.shape)}  "
                f"(expected [1, 3, 256, 288])"
            )
            self.get_logger().info(
                f"[ACT] Post-norm stats: mean={left_t.mean().item():.3f} "
                f"std={left_t.std().item():.3f} "
                f"min={left_t.min().item():.3f} max={left_t.max().item():.3f}"
            )
            self._logged_image_encoding = True

        # --- Process Robot State ---
        # State vector matches training mode (auto-detected in __init__):
        #   - "joints" (6D): joint positions only
        #   - "full" (31D): tcp_pose (7) + tcp_velocity (6) + tcp_error (6)
        #                   + joint_positions (6) + wrench (6)
        joint_positions = _joint_positions_from_obs(obs_msg)

        if self.obs_state_mode == "joints":
            state_np = np.array(joint_positions, dtype=np.float32)
        elif self.obs_state_mode == "joints_port":
            # 6D joints + 3D port_entrance position in base_link frame.
            # Port_entrance is static per trial — cache after first successful lookup.
            port_xyz = self._lookup_port_entrance_baselink(obs_msg)
            state_np = np.array(
                list(joint_positions) + list(port_xyz),
                dtype=np.float32,
            )
        elif self.obs_state_mode == "joints_pose":
            # 6D joints + 9D TCP pose (xyz + 6D rotation).
            tcp_pose = obs_msg.controller_state.tcp_pose
            pose_9d = _pose_to_xyz_rot6d(
                tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z,
                tcp_pose.orientation.x, tcp_pose.orientation.y,
                tcp_pose.orientation.z, tcp_pose.orientation.w,
            )
            state_np = np.array(list(joint_positions) + list(pose_9d), dtype=np.float32)
        elif self.obs_state_mode == "joints_pose_wrench":
            # 6D joints + 9D TCP pose + 6D wrench = 21D. Mirrors mcap_to_lerobot's
            # joints_pose_wrench mode used for Exp-20a training.
            tcp_pose = obs_msg.controller_state.tcp_pose
            pose_9d = _pose_to_xyz_rot6d(
                tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z,
                tcp_pose.orientation.x, tcp_pose.orientation.y,
                tcp_pose.orientation.z, tcp_pose.orientation.w,
            )
            wr = self._read_smoothed_wrench(obs_msg)
            state_np = np.array(
                list(joint_positions) + list(pose_9d) + list(wr),
                dtype=np.float32,
            )
        else:
            tcp_pose = obs_msg.controller_state.tcp_pose
            tcp_vel = obs_msg.controller_state.tcp_velocity
            wr = self._read_smoothed_wrench(obs_msg)
            state_np = np.array(
                [
                    tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z,
                    tcp_pose.orientation.x, tcp_pose.orientation.y,
                    tcp_pose.orientation.z, tcp_pose.orientation.w,
                    tcp_vel.linear.x, tcp_vel.linear.y, tcp_vel.linear.z,
                    tcp_vel.angular.x, tcp_vel.angular.y, tcp_vel.angular.z,
                    *obs_msg.controller_state.tcp_error,
                    *joint_positions,
                    *wr,
                ],
                dtype=np.float32,
            )

        # Normalize State
        raw_state_tensor = (
            torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        )
        obs["observation.state"] = (raw_state_tensor - self.state_mean) / self.state_std

        return obs

    def _read_smoothed_wrench(self, obs_msg):
        """Smoothed 6D wrench at this frame's timestamp, matching training.

        ref_time is the center camera's header stamp — the same anchor
        mcap_to_lerobot uses when building the dataset, so the eval window
        ((ref - window, ref]) is the same set of physical samples that
        trained the model. Falls back to obs_msg.wrist_wrench (single
        sample) only if the smoother buffer is empty.
        """
        stamp = obs_msg.center_image.header.stamp
        ref_time = stamp.sec + stamp.nanosec * 1e-9
        smoothed = self._wrench_smoother.smoothed(
            ref_time,
            window_s=self._wrench_window_s,
            mode=self._wrench_mode,
        )
        if smoothed is None:
            w = obs_msg.wrist_wrench.wrench
            return (
                w.force.x, w.force.y, w.force.z,
                w.torque.x, w.torque.y, w.torque.z,
            )
        return smoothed

    def _lookup_port_entrance_baselink(self, obs_msg):
        """Return port_entrance position in base_link frame as a 3-tuple.

        Cached once per trial. Uses the policy's tf buffer (via parent node).
        In the "joints_port" diagnostic mode, we want a stable, ground-truth
        target to feed the model — testing whether perception is the bottleneck.
        """
        if self._port_entrance_baselink is not None:
            return self._port_entrance_baselink

        try:
            # Find the target port entrance frame from the current task.
            # Format: task_board/<module>/<port>_link_entrance
            task = self._current_task
            frame = f"task_board/{task.target_module_name}/{task.port_name}_link_entrance"
            from rclpy.time import Time
            tr = self._parent_node._tf_buffer.lookup_transform(
                "base_link", frame, Time()
            )
            p = tr.transform.translation
            # Diagnostic perturbation: PORT_OFFSET_{X,Y,Z} env vars add to the
            # ground-truth port position. Used to check whether the policy is
            # actually attending to the port_entrance input — if the output
            # doesn't change when we lie about the port, the model is ignoring
            # this dim.
            import os as _os
            dx = float(_os.environ.get("PORT_OFFSET_X", "0") or 0.0)
            dy = float(_os.environ.get("PORT_OFFSET_Y", "0") or 0.0)
            dz = float(_os.environ.get("PORT_OFFSET_Z", "0") or 0.0)
            self._port_entrance_baselink = (p.x + dx, p.y + dy, p.z + dz)
            self.get_logger().info(
                f"[joints_port] port_entrance in base_link = "
                f"({p.x + dx:.3f}, {p.y + dy:.3f}, {p.z + dz:.3f})"
                + (f"  (perturbed by ({dx:+.3f},{dy:+.3f},{dz:+.3f}))"
                   if (dx or dy or dz) else "")
            )
            return self._port_entrance_baselink
        except Exception as e:
            # Fall back to zeros if TF unavailable (shouldn't happen in practice).
            self.get_logger().warn(f"[joints_port] failed to look up port_entrance: {e}")
            return (0.0, 0.0, 0.0)

    def _eval_alignment(self, task, label: str):
        """Log a multi-component alignment report for approach quality eval.

        Reads ground-truth port + plug TFs (requires launching with
        ground_truth:=true). Computes lateral XY offset, signed Z gap,
        tilt angle between plug and port insertion axes, and a "ready to
        insert" verdict against EVAL_*_OK_* thresholds. One INFO line per
        call with prefix [eval]. Returns the metrics dict or None on TF
        failure (silently after the first warning).
        """
        if not self._eval_gt_log:
            return None
        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        try:
            from rclpy.time import Time
            port_tr = self._parent_node._tf_buffer.lookup_transform(
                "base_link", port_frame, Time()
            ).transform
            plug_tr = self._parent_node._tf_buffer.lookup_transform(
                "base_link", plug_frame, Time()
            ).transform
        except Exception as e:
            if not self._eval_tf_warned:
                self._eval_tf_warned = True
                self.get_logger().warn(
                    f"[eval] TF lookup failed (port={port_frame}, plug={plug_frame}): {e}. "
                    f"Launch with ground_truth:=true to enable EVAL_GT_LOG."
                )
            return None

        pp = port_tr.translation
        lp = plug_tr.translation
        dx, dy, dz = lp.x - pp.x, lp.y - pp.y, lp.z - pp.z
        xy_mm = float(np.sqrt(dx * dx + dy * dy)) * 1000.0
        z_gap_mm = float(dz) * 1000.0
        dist_mm = float(np.sqrt(dx * dx + dy * dy + dz * dz)) * 1000.0

        # Local Z axis of each frame in base_link, derived from the
        # rotation matrix's 3rd column. For quat [x,y,z,w]:
        #   z_axis = (2(xz+yw), 2(yz-xw), 1 - 2(x²+y²))
        def _local_z(rot):
            x, y, z, w = rot.x, rot.y, rot.z, rot.w
            return np.array(
                [2.0 * (x * z + y * w),
                 2.0 * (y * z - x * w),
                 1.0 - 2.0 * (x * x + y * y)],
                dtype=np.float64,
            )

        z_port = _local_z(port_tr.rotation)
        z_plug = _local_z(plug_tr.rotation)
        cos_t = float(np.clip(np.dot(z_port, z_plug), -1.0, 1.0))
        tilt_deg = float(np.degrees(np.arccos(cos_t)))

        xy_ok = xy_mm < self._eval_xy_ok_m * 1000.0
        z_ok = 0.0 < z_gap_mm < self._eval_z_ok_m * 1000.0
        tilt_ok = tilt_deg < self._eval_tilt_ok_deg
        ready = xy_ok and z_ok and tilt_ok
        if ready:
            verdict = "READY"
        else:
            reasons = []
            if not xy_ok:
                reasons.append("xy_high")
            if not z_ok:
                reasons.append("z_high" if z_gap_mm >= self._eval_z_ok_m * 1000.0 else "z_below_port")
            if not tilt_ok:
                reasons.append("tilt_high")
            verdict = f"NOT_READY ({', '.join(reasons)})"

        self.get_logger().info(
            f"[eval] {label}: "
            f"xy={xy_mm:6.2f}mm z={z_gap_mm:+7.2f}mm tilt={tilt_deg:5.1f}° "
            f"dist={dist_mm:7.2f}mm  {verdict}"
        )
        return {
            "xy_mm": xy_mm,
            "z_gap_mm": z_gap_mm,
            "tilt_deg": tilt_deg,
            "dist_mm": dist_mm,
            "ready": ready,
        }

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        # Reset per-trial port_entrance cache (joints_port diagnostic mode).
        self._port_entrance_baselink = None
        self._current_task = task

        # Inference loop parameters
        # - CONTROL_HZ is interpreted as **sim Hz** (loop is paced via
        #   self.sleep_for / self.time_now, both of which read the node's
        #   sim-time-aware clock). Should match the training-dataset fps so
        #   the (state, action) cadence at inference matches what the model
        #   was trained on. Default 20 sim Hz to match our 20-Hz datasets.
        # - All time arithmetic in this loop uses sim time. Wall-time clock
        #   has been removed because under host load (or RTF != 1) it
        #   silently drifts the policy's effective rate vs sim, breaking
        #   the train/eval temporal correspondence and miscalibrating
        #   ACT_TIME_LIMIT_CAP.
        # - RESET_EVERY forces a fresh chunk prediction every N iterations.
        # - ACT_TEMPORAL_ENSEMBLE controls temporal ensembling (default: 0.1).
        #   Set to empty string or "off" to disable.
        CONTROL_HZ = float(os.environ.get("ACT_CONTROL_HZ", "20.0"))
        CONTROL_DT = 1.0 / CONTROL_HZ
        RESET_EVERY = int(os.environ.get("ACT_RESET_EVERY", "10"))

        # Temporal ensembling: default 0.1 (tuned for the AIC SFP task in
        # Exp-29 — TE=0.1 hits 10F/2P/17M/1W vs the paper's 0.01 at 6F/3P).
        # Set ACT_TEMPORAL_ENSEMBLE=off to disable and use RESET_EVERY instead.
        te_coeff_str = os.environ.get("ACT_TEMPORAL_ENSEMBLE", "0.2")
        use_temporal_ensemble = te_coeff_str not in ("", "off", "0")
        if use_temporal_ensemble:
            te_coeff = float(te_coeff_str)
            if self.policy.config.temporal_ensemble_coeff is None:
                self.policy.config.temporal_ensemble_coeff = te_coeff
                self.policy.config.n_action_steps = 1
                from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
                self.policy.temporal_ensembler = ACTTemporalEnsembler(
                    te_coeff, self.policy.config.chunk_size
                )

        self.policy.reset()
        self.get_logger().info(
            f"RunACT.insert_cable() enter. Task: {task} "
            f"temporal_ensemble={'%.3f' % te_coeff if use_temporal_ensemble else 'off'} "
            f"reset_every={RESET_EVERY if not use_temporal_ensemble else 'N/A'}"
        )
        self._eval_alignment(task, "start")

        # Optional video recording: set ACT_VIDEO_DIR to enable.
        # Writes center camera footage (downsampled to match training input)
        # as mp4 — one file per insert_cable call, named
        # <timestamp>_<model-tag>_<module>_<plug>.mp4
        # Set ACT_MODEL_TAG to include a model identifier in the filename.
        video_writer = None
        video_dir_env = os.environ.get("ACT_VIDEO_DIR", "")
        if video_dir_env:
            video_dir = Path(video_dir_env)
            video_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            model_tag = os.environ.get("ACT_MODEL_TAG", "model")
            video_path = video_dir / f"{ts}_{model_tag}_{task.target_module_name}_{task.plug_name}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            # Dimensions: raw 1152x1024 * 0.25 → 288x256
            video_writer = cv2.VideoWriter(
                str(video_path), fourcc, CONTROL_HZ, (288, 256)
            )
            self.get_logger().info(f"[VIDEO] Recording center camera → {video_path}")

        time_limit = min(
            float(getattr(task, "time_limit", 180)),
            float(os.environ.get("ACT_TIME_LIMIT_CAP", "180")),
        )

        # Sim-time bound for the policy's intent (loop cadence stays sim-time
        # for train/eval correspondence) AND a wall-time bound to beat the
        # engine's wall-clock cancel deadline. aic_engine cancels at
        # task.time_limit WALL seconds (chrono::seconds, NOT sim-time-aware).
        # When sim runs <1x realtime, RunACT's sim-bounded loop runs longer
        # in wall time than the engine's wait → engine cancels mid-loop and
        # the trial is scored as TimeLimitExceeded. We can't patch the
        # organizer's engine, so RunACT exits ACT_WALL_SAFETY_S wall seconds
        # before the engine's deadline. Default 10s is generous; lower if
        # task.time_limit is short.
        wall_safety_s = float(os.environ.get("ACT_WALL_SAFETY_S", "10.0"))
        wall_limit_s = max(0.0,
                           float(getattr(task, "time_limit", 180)) - wall_safety_s)

        # All loop timekeeping is sim-time (self.time_now / self.sleep_for).
        # time_limit is interpreted as sim seconds.
        start_time = self.time_now()
        start_wall = time.monotonic()
        iter_count = 0

        def _should_stop() -> bool:
            sim_elapsed = (self.time_now() - start_time).nanoseconds * 1e-9
            if sim_elapsed >= time_limit:
                return True
            wall_elapsed = time.monotonic() - start_wall
            if wall_elapsed >= wall_limit_s:
                self.get_logger().info(
                    f"RunACT: wall-time guard fired at sim={sim_elapsed:.1f}s "
                    f"wall={wall_elapsed:.1f}s (limit {wall_limit_s:.1f}s wall, "
                    f"engine deadline {getattr(task, 'time_limit', 180)}s wall)"
                )
                return True
            return False

        while not _should_stop():
            loop_start = self.time_now()

            # 1. Get & Process Observation
            observation_msg = get_observation()

            if observation_msg is None:
                self.get_logger().info("No observation received.")
                break

            tcp = observation_msg.controller_state.tcp_pose

            # Write center camera frame to video if recording
            if video_writer is not None:
                raw_img = observation_msg.center_image
                h, w = raw_img.height, raw_img.width
                enc = raw_img.encoding.lower() if raw_img.encoding else "rgb8"
                if enc == "rgb8":
                    img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 3)
                elif enc == "bgr8":
                    raw = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 3)
                    img_np = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
                else:
                    img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(h, w, 3)
                img_small = cv2.resize(img_np, (288, 256), interpolation=cv2.INTER_AREA)
                # cv2 VideoWriter expects BGR
                video_writer.write(cv2.cvtColor(img_small, cv2.COLOR_RGB2BGR))

            # Periodic state diagnostic
            if iter_count % 100 == 0:
                wrn = observation_msg.wrist_wrench.wrench
                err = observation_msg.controller_state.tcp_error
                self.get_logger().info(
                    f"[STATE] iter={iter_count} "
                    f"tcp=({tcp.position.x:.3f},{tcp.position.y:.3f},{tcp.position.z:.3f}) "
                    f"wrench=({wrn.force.x:.1f},{wrn.force.y:.1f},{wrn.force.z:.1f}) "
                    f"tcp_err=({err[0]:.4f},{err[1]:.4f},{err[2]:.4f})"
                )

            # Force a fresh chunk periodically so the policy reacts to new
            # observations. Skipped when temporal ensembling is active (it
            # re-runs the model every step internally).
            if not use_temporal_ensemble and iter_count > 0 and iter_count % RESET_EVERY == 0:
                self.policy.reset()

            action, norm_act_np = self._act_step(observation_msg, move_robot)

            if iter_count % 100 == 0:
                self.get_logger().info(
                    f"[ACTION] mode={self.action_mode} "
                    f"norm={np.round(norm_act_np, 3)} "
                    f"raw={np.round(action, 4)}"
                )
            if iter_count % 25 == 0:
                self._eval_alignment(task, f"iter={iter_count}")
            send_feedback("in progress...")

            iter_count += 1

            elapsed = (self.time_now() - loop_start).nanoseconds * 1e-9
            self.sleep_for(max(0.0, CONTROL_DT - elapsed))

        if video_writer is not None:
            video_writer.release()
            self.get_logger().info(f"[VIDEO] Saved to {video_path}")

        elapsed_sim_s = (self.time_now() - start_time).nanoseconds * 1e-9
        self.get_logger().info(
            f"RunACT.insert_cable() exiting after {iter_count} iterations "
            f"({elapsed_sim_s:.1f}s sim)..."
        )
        self._eval_alignment(task, "final")
        return True

    def _act_step(self, observation_msg, move_robot):
        """One ACT inference step: tensorize obs, forward, un-normalize, dispatch.

        Returns (action, normalized_action_np) so callers can log/inspect.
        Used by RunACT.insert_cable's inner loop and by RunACTThenInsert,
        which interleaves ACT inference with a contact-force handoff check.
        """
        tcp = observation_msg.controller_state.tcp_pose
        obs_tensors = self.prepare_observations(observation_msg)

        with torch.inference_mode():
            normalized_action = self.policy.select_action(obs_tensors)

        if self.action_norm_mode == "MINMAX":
            raw_action_tensor = (normalized_action + 1) / 2 * (self.action_max - self.action_min) + self.action_min
        else:
            raw_action_tensor = (normalized_action * self.action_std) + self.action_mean
        action = raw_action_tensor[0].cpu().numpy()
        norm_act_np = normalized_action[0].cpu().numpy()

        # ACT_SPEED_FACTOR scaling. speed==1.0 reproduces the model's action
        # verbatim; <1.0 interpolates current→predicted (absolute modes) or
        # scales the delta (twist mode). See module-level docstring.
        speed = self._speed_factor

        if self.action_mode == "joint":
            predicted_q = np.asarray(action[:6], dtype=np.float64)
            if speed == 1.0:
                target_q = predicted_q
            else:
                current_q = np.asarray(
                    _joint_positions_from_obs(observation_msg), dtype=np.float64
                )
                target_q = current_q + speed * (predicted_q - current_q)
            joint_update = self._make_joint_motion_update(target_q)
            move_robot(joint_motion_update=joint_update)
        elif self.action_mode == "cartesian_pose":
            px, py, pz = float(action[0]), float(action[1]), float(action[2])
            pq = np.asarray(_rot6d_to_quat_xyzw(action[3:6], action[6:9]),
                            dtype=np.float64)
            if speed == 1.0:
                tx, ty, tz = px, py, pz
                tq = pq
            else:
                cx, cy, cz = tcp.position.x, tcp.position.y, tcp.position.z
                cq = np.array([tcp.orientation.x, tcp.orientation.y,
                               tcp.orientation.z, tcp.orientation.w],
                              dtype=np.float64)
                tx = cx + speed * (px - cx)
                ty = cy + speed * (py - cy)
                tz = cz + speed * (pz - cz)
                tq = _quaternion_slerp_xyzw(cq, pq, speed)
            target_pose = Pose(
                position=Point(x=tx, y=ty, z=tz),
                orientation=Quaternion(
                    x=float(tq[0]), y=float(tq[1]),
                    z=float(tq[2]), w=float(tq[3]),
                ),
            )
            move_robot(motion_update=self._make_pose_motion_update(target_pose))
        else:
            # Cartesian (twist delta): scale position and rotation deltas
            target_pos = Point(
                x=tcp.position.x + speed * float(action[0]),
                y=tcp.position.y + speed * float(action[1]),
                z=tcp.position.z + speed * float(action[2]),
            )
            rotvec = speed * action[3:6].astype(np.float64)
            angle = np.linalg.norm(rotvec)
            if angle > 1e-10:
                axis = rotvec / angle
                half = angle / 2.0
                dq = np.array([axis[0]*np.sin(half), axis[1]*np.sin(half),
                               axis[2]*np.sin(half), np.cos(half)])
            else:
                dq = np.array([0.0, 0.0, 0.0, 1.0])
            cq = np.array([tcp.orientation.x, tcp.orientation.y,
                           tcp.orientation.z, tcp.orientation.w])
            v1, w1 = cq[:3], cq[3]
            v2, w2 = dq[:3], dq[3]
            tw = w1*w2 - np.dot(v1, v2)
            tv = w1*v2 + w2*v1 + np.cross(v1, v2)
            tq = np.array([tv[0], tv[1], tv[2], tw])
            tq /= np.linalg.norm(tq)
            target_ori = Quaternion(x=float(tq[0]), y=float(tq[1]),
                                    z=float(tq[2]), w=float(tq[3]))
            target_pose = Pose(position=target_pos, orientation=target_ori)
            move_robot(motion_update=self._make_pose_motion_update(target_pose))

        return action, norm_act_np

    def _make_pose_motion_update(self, pose: Pose, frame_id: str = "base_link"):
        """Build a MODE_POSITION MotionUpdate for the given absolute pose.

        Uses the same stiffness/damping as CheatCode (the scripted policy that
        generated the training data) so the impedance behaviour matches what the
        model learned from.
        """
        return MotionUpdate(
            header=Header(
                frame_id=frame_id,
                stamp=self.get_clock().now().to_msg(),
            ),
            pose=pose,
            target_stiffness=np.diag(
                [90.0, 90.0, 90.0, 50.0, 50.0, 50.0]
            ).flatten(),
            target_damping=np.diag(
                [50.0, 50.0, 50.0, 20.0, 20.0, 20.0]
            ).flatten(),
            feedforward_wrench_at_tip=Wrench(
                force=Vector3(x=0.0, y=0.0, z=0.0),
                torque=Vector3(x=0.0, y=0.0, z=0.0),
            ),
            wrench_feedback_gains_at_tip=[0.5, 0.5, 0.5, 0.0, 0.0, 0.0],
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION,
            ),
        )

    def _make_joint_motion_update(self, joint_positions: np.ndarray):
        """Build a JointMotionUpdate for absolute joint position targets."""
        point = JointTrajectoryPoint()
        point.positions = [float(q) for q in joint_positions[:6]]
        return JointMotionUpdate(
            target_state=point,
            target_stiffness=[100.0] * 6,
            target_damping=[40.0] * 6,
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION,
            ),
        )
