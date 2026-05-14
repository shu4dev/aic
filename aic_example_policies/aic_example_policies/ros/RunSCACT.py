"""SC-task variant of RunACT.

Only deviation from RunACT: the Cartesian impedance sent with each pose
command matches the stiffness/damping SCCheatCode used at data-collection
time (400/200 instead of RunACT's 90/50 default). mcap_to_lerobot does not
record stiffness in the action label, so the trained model's pose
trajectories were implicitly calibrated against a 400/200 controller;
running them through 90/50 yields ~4x weaker lateral force and the PI
integrator cannot drag the cable through SC's alignment sleeve. SFP
rollouts continue to use RunACT directly.
"""

import numpy as np

from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_example_policies.ros.RunACT import RunACT
from geometry_msgs.msg import Pose, Vector3, Wrench
from std_msgs.msg import Header


class RunSCACT(RunACT):
    SC_STIFFNESS = [400.0, 400.0, 400.0, 150.0, 150.0, 150.0]
    SC_DAMPING = [200.0, 200.0, 200.0, 60.0, 60.0, 60.0]

    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        self.get_logger().info(
            f"[RunSCACT] SC impedance: K={self.SC_STIFFNESS} D={self.SC_DAMPING}"
        )
        return super().insert_cable(
            task, get_observation, move_robot, send_feedback
        )

    def _make_pose_motion_update(self, pose: Pose, frame_id: str = "base_link"):
        return MotionUpdate(
            header=Header(
                frame_id=frame_id,
                stamp=self.get_clock().now().to_msg(),
            ),
            pose=pose,
            target_stiffness=np.diag(self.SC_STIFFNESS).flatten(),
            target_damping=np.diag(self.SC_DAMPING).flatten(),
            feedforward_wrench_at_tip=Wrench(
                force=Vector3(x=0.0, y=0.0, z=0.0),
                torque=Vector3(x=0.0, y=0.0, z=0.0),
            ),
            wrench_feedback_gains_at_tip=[0.5, 0.5, 0.5, 0.0, 0.0, 0.0],
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION,
            ),
        )
