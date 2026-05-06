"""DART-style noise injection wrapper around ImprovedCheatCode.

Goal: generate training data for a downstream ACT policy that covers a broader
state distribution than the nominal expert produces, so ACT learns to recover
from states it will visit at test time but the un-perturbed expert never does.

Design choices:

1. Noise is injected on the *output* of `_calc_pose`, by overriding
   `set_pose_target`. This is correct because the parent's PI integrator reads
   port/plug TF directly; perturbing the TF readings would corrupt the error
   term. Perturbing the commanded pose lets the integrator wind up against the
   disturbance and produce real corrective demonstrations -- which is the
   property DART relies on.

2. Noise magnitude is *state-dependent*, not phase-tagged. We compute the
   current plug-to-port z-gap and xy-error from TF and scale noise accordingly:
   large gaps = approach territory = generous noise;
   small gaps = descent territory = minimal noise.
   This avoids needing to track phase across the parent's `_align_and_descend`
   method (which would require copy-paste duplication of 200 lines).

3. Noise is *temporally correlated*. ACT applies temporal ensembling at
   inference, which averages out high-frequency noise. To get demonstrations
   that broaden the *state* distribution (not just the action distribution),
   we resample noise every N control steps and hold it. Per-step IID noise
   would be smoothed away by ACT and provide little training value.

4. A hard kill-switch (`AIC_NOISE=0`) disables noise entirely. The submitted
   ACT container must never have noise active at inference -- this wrapper is
   only used during training-data generation.

5. End-of-trial joint hold (`hold_cmd` in the parent's `insert_cable`) goes
   through `move_robot(joint_motion_update=...)`, not `set_pose_target`, so
   it is naturally exempt from noise. Good.

Usage: register `NoisyImprovedCheatCode` as the policy class via the
aic_model parameter system, same as `ImprovedCheatCode`. Set env vars to
control noise behavior.
"""

import math
import os

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion

from aic_example_policies.ros.ImprovedCheatCode import ImprovedCheatCode


class NoisyImprovedCheatCode(ImprovedCheatCode):
    """Noise-augmented expert for DART-style training data generation.

    Environment variables:
      AIC_NOISE=0            -- disable noise (use for evaluation/submission)
      AIC_NOISE_SEED=<int>   -- RNG seed; default = trial-randomized
      AIC_NOISE_SCALE=<f>    -- global multiplier on all noise magnitudes
                                (default 1.0; use 0.5 for conservative, 2.0 to
                                 stress-test diversity)
      AIC_NOISE_HOLD=<int>   -- control steps between noise resamples
                                (default 8 ~ 320ms at 25Hz)
      AIC_NOISE_LOG=1        -- log injected noise per step for debugging
    """

    # Noise schedule: keys are upper z-gap bounds (m); values are stddevs.
    # xy is lateral (m), z is vertical (m), rot is per-axis rotation (rad).
    # The schedule is consulted by current plug->port z-gap; the first bucket
    # whose upper bound exceeds the gap wins.
    NOISE_SCHEDULE = [
        # (z_gap_upper_bound_m, xy_std, z_std, rot_std)
        (0.005,   0.00010, 0.0,     0.001),  # near contact -- minimal
        (0.015,   0.00040, 0.0,     0.003),  # late align / pre-descend
        (0.040,   0.00120, 0.0010,  0.008),  # align body
        (0.080,   0.00300, 0.0020,  0.020),  # approach
        (math.inf, 0.00500, 0.0030,  0.030), # far approach / pre-task
    ]

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self._noise_enabled = os.environ.get("AIC_NOISE", "1") != "0"
        seed = os.environ.get("AIC_NOISE_SEED")
        self._rng = np.random.default_rng(int(seed) if seed else None)
        self._noise_scale = float(os.environ.get("AIC_NOISE_SCALE", "1.0"))
        self._noise_hold = int(os.environ.get("AIC_NOISE_HOLD", "8"))
        self._noise_log = os.environ.get("AIC_NOISE_LOG", "0") == "1"

        # Held noise sample, refreshed every _noise_hold calls.
        self._held_noise = None  # (dx, dy, dz, drx, dry, drz)
        self._noise_step = 0

        if self._noise_enabled:
            self.get_logger().info(
                f"[NoisyICC] noise ENABLED scale={self._noise_scale} "
                f"hold={self._noise_hold} steps"
            )
        else:
            self.get_logger().info("[NoisyICC] noise DISABLED (AIC_NOISE=0)")

    # ------------------------------------------------------------------
    # Noise generation
    # ------------------------------------------------------------------

    def _current_z_gap(self):
        """Plug-above-port z-gap in meters. Returns inf if TF unavailable."""
        port = self._port_tf()
        plug = self._plug_tf()
        if port is None or plug is None:
            return math.inf
        return plug.translation.z - port.translation.z

    def _noise_stddevs(self):
        """Pick (xy_std, z_std, rot_std) from the schedule based on z-gap."""
        gap = self._current_z_gap()
        for upper, xy_s, z_s, rot_s in self.NOISE_SCHEDULE:
            if gap <= upper:
                return (
                    xy_s * self._noise_scale,
                    z_s * self._noise_scale,
                    rot_s * self._noise_scale,
                )
        return (0.0, 0.0, 0.0)

    def _sample_noise(self):
        """Sample a noise vector and cache it for _noise_hold steps."""
        if self._held_noise is not None and self._noise_step < self._noise_hold:
            self._noise_step += 1
            return self._held_noise

        xy_std, z_std, rot_std = self._noise_stddevs()
        # Sample lateral noise as 2D Gaussian, vertical separately.
        dx, dy = self._rng.normal(0.0, xy_std, size=2)
        dz = self._rng.normal(0.0, z_std)
        drx, dry, drz = self._rng.normal(0.0, rot_std, size=3)

        self._held_noise = (dx, dy, dz, drx, dry, drz)
        self._noise_step = 1

        if self._noise_log:
            self.get_logger().info(
                f"[NoisyICC] resample dxy={math.sqrt(dx*dx+dy*dy)*1000:.2f}mm "
                f"dz={dz*1000:.2f}mm drot={math.degrees(math.sqrt(drx*drx+dry*dry+drz*drz)):.2f}deg"
            )
        return self._held_noise

    @staticmethod
    def _quat_mul(q1, q2):
        """Hamilton product (w, x, y, z)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        )

    @staticmethod
    def _small_rotation_quat(rx, ry, rz):
        """Quaternion from a small rotation vector (axis-angle, ~radians)."""
        angle = math.sqrt(rx*rx + ry*ry + rz*rz)
        if angle < 1e-9:
            return (1.0, 0.0, 0.0, 0.0)
        s = math.sin(0.5 * angle) / angle
        return (math.cos(0.5 * angle), rx * s, ry * s, rz * s)

    def _perturb_pose(self, pose: Pose) -> Pose:
        dx, dy, dz, drx, dry, drz = self._sample_noise()

        new_pos = Point(
            x=pose.position.x + dx,
            y=pose.position.y + dy,
            z=pose.position.z + dz,
        )

        q_orig = (pose.orientation.w, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z)
        q_noise = self._small_rotation_quat(drx, dry, drz)
        # Apply noise on the right (in TCP/local frame) so the perturbation
        # is body-relative rather than world-relative. This keeps the
        # gripper's "forward" axis pointing roughly at the port even with
        # large perturbations.
        w, x, y, z = self._quat_mul(q_orig, q_noise)
        new_orient = Quaternion(w=w, x=x, y=y, z=z)

        return Pose(position=new_pos, orientation=new_orient)

    # ------------------------------------------------------------------
    # Override
    # ------------------------------------------------------------------

    def set_pose_target(self, move_robot, pose, frame_id="base_link",
                        stiffness=None, damping=None):
        if self._noise_enabled:
            pose = self._perturb_pose(pose)
        # Forward to parent with whatever kwargs were set; preserve defaults
        # by only passing through what was supplied.
        kwargs = {"frame_id": frame_id}
        if stiffness is not None:
            kwargs["stiffness"] = stiffness
        if damping is not None:
            kwargs["damping"] = damping
        super().set_pose_target(move_robot=move_robot, pose=pose, **kwargs)

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        # Reset the held-noise cache at the start of every trial so trials
        # don't share noise state.
        self._held_noise = None
        self._noise_step = 0
        return super().insert_cable(task, get_observation, move_robot, send_feedback)